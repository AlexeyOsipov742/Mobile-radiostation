#include "TxRx.h"

#include <atomic>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

// ------------------------------------------------------------
// LCD 20x4 over I2C (PCF8574 @ 0x27). Very small driver.
// Defaults for Raspberry Pi: /dev/i2c-1, addr 0x27.
// Can be overridden with env:
//   SCREEN_I2C_DEV=/dev/i2c-1
//   SCREEN_I2C_ADDR=0x27
// ------------------------------------------------------------

namespace {

// PCF8574 pins
constexpr uint8_t LCD_RS = 0x01;  // P0
constexpr uint8_t LCD_EN = 0x04;  // P2
constexpr uint8_t LCD_BL = 0x08;  // P3 (active high)
constexpr uint8_t LCD_RW = 0x00;  // RW not used

constexpr uint8_t LINE_ADDR[4] = {0x80, 0xC0, 0x94, 0xD4};

static inline void usleep_safe(int us) {
    if (us > 0) ::usleep((useconds_t)us);
}

struct Lcd20x4 {
    int fd = -1;
    uint8_t bl = LCD_BL;

    bool open_dev(const char* dev, int addr) {
        fd = ::open(dev, O_RDWR);
        if (fd < 0) return false;
        if (ioctl(fd, I2C_SLAVE, addr) < 0) {
            ::close(fd);
            fd = -1;
            return false;
        }
        return true;
    }

    void close_dev() {
        if (fd >= 0) ::close(fd);
        fd = -1;
    }

    bool ok() const { return fd >= 0; }

    bool write8(uint8_t v) {
        if (fd < 0) return false;
        if (::write(fd, &v, 1) != 1) return false;
        return true;
    }

    void pulse(uint8_t d) {
        write8(d | bl | LCD_EN);
        usleep_safe(1);
        write8((d | bl) & ~LCD_EN);
        usleep_safe(50);
    }

    void send_nibble(uint8_t nib, uint8_t mode_rs) {
        uint8_t d = (nib & 0xF0) | bl | mode_rs | LCD_RW;
        write8(d);
        pulse(d);
    }

    void send_byte(uint8_t val, uint8_t mode_rs) {
        send_nibble(val & 0xF0, mode_rs);
        send_nibble((uint8_t)((val << 4) & 0xF0), mode_rs);
    }

    void cmd(uint8_t c) {
        send_byte(c, 0);
        if (c == 0x01 || c == 0x02) usleep_safe(2000);
    }

    void data(uint8_t c) {
        send_byte(c, LCD_RS);
    }

    void init() {
        // init sequence for 4-bit mode
        usleep_safe(50000);
        send_nibble(0x30, 0);
        usleep_safe(4500);
        send_nibble(0x30, 0);
        usleep_safe(4500);
        send_nibble(0x30, 0);
        usleep_safe(150);
        send_nibble(0x20, 0);      // 4-bit

        cmd(0x28);                 // 4-bit, 2-line (works for 20x4 too), 5x8
        cmd(0x0C);                 // display ON, cursor OFF
        cmd(0x06);                 // entry mode
        cmd(0x01);                 // clear
    }

    void set_cursor(uint8_t row, uint8_t col) {
        if (row > 3) row = 3;
        cmd((uint8_t)(LINE_ADDR[row] + col));
    }

    void write_line(uint8_t row, const std::string& s) {
        if (!ok()) return;
        std::string out = s;
        if ((int)out.size() < 20) out.append(20 - out.size(), ' ');
        if ((int)out.size() > 20) out.resize(20);

        set_cursor(row, 0);
        for (char c : out) data((uint8_t)c);
    }
};

static inline void rtrim_spaces(std::string& s) {
    while (!s.empty() && s.back() == ' ') s.pop_back();
}

static inline void replace_caret_with_space(std::string& s) {
    for (char& c : s) if (c == '^') c = ' ';
}

// Softkeys line: 5 buttons → 5 blocks × 4 chars = 20 chars.
// Input often is like: "^ZNUP^MON ^COLR^PWR ^ZNDN"
static std::string format_softkeys_5x4(std::string s) {
    // keep carets for splitting; only trim outer spaces
    while (!s.empty() && s.front() == ' ') s.erase(s.begin());
    while (!s.empty() && s.back()  == ' ') s.pop_back();

    // split by '^'
    std::vector<std::string> tok;
    tok.reserve(8);

    std::string cur;
    for (char c : s) {
        if (c == '^') {
            if (!cur.empty()) {
                // trim
                while (!cur.empty() && cur.front() == ' ') cur.erase(cur.begin());
                while (!cur.empty() && cur.back()  == ' ') cur.pop_back();
                if (!cur.empty()) tok.push_back(cur);
            }
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) {
        while (!cur.empty() && cur.front() == ' ') cur.erase(cur.begin());
        while (!cur.empty() && cur.back()  == ' ') cur.pop_back();
        if (!cur.empty()) tok.push_back(cur);
    }

    if (tok.empty()) {
        // fallback: just replace carets with spaces and clamp
        replace_caret_with_space(s);
        if ((int)s.size() > 20) s.resize(20);
        return s;
    }

    // Take first 5 tokens, each padded/truncated to 4
    std::string out;
    out.reserve(20);
    for (int i = 0; i < 5; ++i) {
        std::string t = (i < (int)tok.size()) ? tok[i] : "";
        replace_caret_with_space(t);
        if ((int)t.size() > 4) t.resize(4);
        if ((int)t.size() < 4) t.append(4 - t.size(), ' ');
        out += t;
    }
    if ((int)out.size() > 20) out.resize(20);
    return out;
}

struct ScreenState {
    std::string line[4];
};

// Extract SBEP-ish display update blocks.
// Empirically for XTL front panel we see blocks that contain:
//   1F 00 .. .. .. .. row 00 [ASCII...]
// where row = 0,1,2 correspond to the 3 text lines we want.
//
// We DO NOT convert the whole packet to ASCII; we only extract
// text from these 1F 00 blocks to avoid garbage like \"Wg\".
static void apply_display_updates(const uint8_t* data, int len, ScreenState& st, Lcd20x4& lcd) {
    for (int i = 0; i + 10 <= len; ++i) {
        if (data[i] != 0x1F || data[i + 1] != 0x00) continue;

        // row index (based on your captured frames)
        uint8_t row = data[i + 6];
        if (row > 3) continue;

        // text starts at i+8
        int j = i + 8;
        std::string text;
        text.reserve(32);

        // read until 0x00 padding or until non-printable long gap
        // but allow spaces and '^'
        for (; j < len; ++j) {
            uint8_t c = data[j];
            if (c == 0x00) break;
            if (c == '^' || c == ' ' || (c >= 0x20 && c <= 0x7E)) {
                text.push_back((char)c);
                if ((int)text.size() >= 40) break; // hard cap
            } else {
                // stop if we've already started collecting
                if (!text.empty()) break;
            }
        }

        rtrim_spaces(text);
        if (text.empty()) continue;

        // Row2 = softkeys; render as 5×4 to fit 20 chars.
        std::string shown = text;
        if (row == 2) {
            shown = format_softkeys_5x4(text);
        } else {
            // For other rows: just replace '^' with space (rare but safe)
            replace_caret_with_space(shown);
        }

        // clamp for LCD
        if ((int)shown.size() > 20) shown.resize(20);

        if (st.line[row] != shown) {
            st.line[row] = shown;
            lcd.write_line((uint8_t)row, shown);

            // optional terminal log (kept short)
            std::printf("LCD row%u: %s\n", row + 1, shown.c_str());
        }
    }
}

static int make_listen_socket(int port) {
    int s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return -1;

    int opt = 1;
    ::setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons((uint16_t)port);

    if (::bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) {
        ::close(s);
        return -1;
    }
    if (::listen(s, 8) < 0) {
        ::close(s);
        return -1;
    }
    return s;
}

static bool read_exact(int fd, void* out, size_t n) {
    uint8_t* p = (uint8_t*)out;
    while (n > 0) {
        ssize_t r = ::recv(fd, p, n, 0);
        if (r == 0) return false;
        if (r < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        p += (size_t)r;
        n -= (size_t)r;
    }
    return true;
}

} // namespace

void command(std::atomic<bool> &running) {
    // LCD init
    const char* dev = std::getenv("SCREEN_I2C_DEV");
    if (!dev) dev = "/dev/i2c-1";
    const char* addr_s = std::getenv("SCREEN_I2C_ADDR");
    int addr = 0x27;
    if (addr_s && *addr_s) addr = (int)std::strtol(addr_s, nullptr, 0);

    Lcd20x4 lcd;
    if (lcd.open_dev(dev, addr)) {
        lcd.init();
        lcd.write_line(0, "Remote UI online");
        lcd.write_line(1, "Waiting screen...");
        lcd.write_line(2, "");
        lcd.write_line(3, "");
        std::printf("[CMD] LCD OK: %s addr=0x%02X\n", dev, addr);
    } else {
        std::fprintf(stderr, "[CMD] LCD init failed (%s addr=0x%02X). Continuing without LCD.\n", dev, addr);
    }

    // TCP server for display frames (len_le + payload), port 7777 (as in debug_server)
    constexpr int kPort = 7777;
    int ls = make_listen_socket(kPort);
    if (ls < 0) {
        std::perror("[CMD] bind/listen 7777");
        lcd.close_dev();
        return;
    }
    std::printf("[CMD] Screen RX listening on :%d\n", kPort);

    ScreenState st;
    for (auto& s : st.line) s.clear();

    std::vector<uint8_t> payload;
    payload.resize(4096);

    while (running) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(ls, &rfds);
        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 200 * 1000; // 200ms tick

        int r = ::select(ls + 1, &rfds, nullptr, nullptr, &tv);
        if (r <= 0) continue;

        sockaddr_in cli{};
        socklen_t cl = sizeof(cli);
        int cs = ::accept(ls, (sockaddr*)&cli, &cl);
        if (cs < 0) continue;

        // frame: [uint16_le len] [payload]
        uint16_t len_le = 0;
        if (!read_exact(cs, &len_le, sizeof(len_le))) {
            ::close(cs);
            continue;
        }
        uint16_t len = le16toh(len_le);
        if (len == 0 || len > 4096) {
            // drain (best effort) then drop
            uint8_t tmp[256];
            while (::recv(cs, tmp, sizeof(tmp), 0) > 0) {}
            ::close(cs);
            continue;
        }
        if (!read_exact(cs, payload.data(), len)) {
            ::close(cs);
            continue;
        }
        ::close(cs);

        apply_display_updates(payload.data(), (int)len, st, lcd);
    }

    ::close(ls);
    lcd.close_dev();
    std::printf("[CMD] Screen RX stopped\n");
}

