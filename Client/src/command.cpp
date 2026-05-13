#include "TxRx.h"

#include <algorithm>
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

struct SbepMsg {
    uint16_t opcode = 0;
    std::vector<uint8_t> data;
};

enum class ParseRes {
    Ok,
    Incomplete,
    Invalid
};

static bool sbep_checksum_ok(const uint8_t* msg, size_t n) {
    if (n < 2) return true;
    uint32_t sum = 0;
    for (size_t i = 0; i + 1 < n; ++i) sum += msg[i];
    const uint8_t expect = (uint8_t)(0xFF - (sum & 0xFF));
    return expect == msg[n - 1];
}

static ParseRes sbep_try_head(const std::vector<uint8_t>& stream,
                              size_t& out_total_len,
                              uint16_t& out_opcode,
                              size_t& out_data_start,
                              size_t& out_data_len) {
    if (stream.empty()) return ParseRes::Incomplete;

    const uint8_t b0 = stream[0];
    const uint8_t msn = (b0 >> 4) & 0x0F;
    const uint8_t lsn = (b0 & 0x0F);
    size_t idx = 1;

    const bool ext_opcode = (msn == 0x0F);
    const bool ext_size = (lsn == 0x0F);

    if (!ext_opcode) {
        out_opcode = msn;
    } else {
        if (idx >= stream.size()) return ParseRes::Incomplete;
        out_opcode = stream[idx];
        idx += 1;
    }

    uint32_t follow = 0;
    if (!ext_size) {
        follow = lsn;
    } else {
        if (idx + 1 >= stream.size()) return ParseRes::Incomplete;
        follow = ((uint16_t)stream[idx] << 8) | (uint16_t)stream[idx + 1];
        idx += 2;
    }

    if (follow > 4096) return ParseRes::Invalid;
    const size_t total = idx + (size_t)follow;
    if (stream.size() < total) return ParseRes::Incomplete;

    const bool has_ck = (follow > 0);
    const size_t data_end = total - (has_ck ? 1 : 0);

    out_total_len = total;
    out_data_start = idx;
    out_data_len = (data_end > idx) ? (data_end - idx) : 0;

    if (has_ck && !sbep_checksum_ok(stream.data(), total)) return ParseRes::Invalid;
    return ParseRes::Ok;
}

static bool sbep_extract_next(std::vector<uint8_t>& stream, SbepMsg& out) {
    const size_t kMaxKeep = 64 * 1024;
    const size_t kTailKeep = 8 * 1024;

    if (stream.empty()) return false;

    while (!stream.empty()) {
        size_t total_len = 0;
        uint16_t opcode = 0;
        size_t data_start = 0;
        size_t data_len = 0;

        ParseRes pr = sbep_try_head(stream, total_len, opcode, data_start, data_len);
        if (pr == ParseRes::Incomplete) {
            if (stream.size() > kMaxKeep && stream.size() > kTailKeep) {
                stream.erase(stream.begin(), stream.end() - (long)kTailKeep);
            }
            return false;
        }
        if (pr == ParseRes::Invalid) {
            stream.erase(stream.begin());
            continue;
        }

        out.opcode = opcode;
        out.data.assign(stream.begin() + (long)data_start, stream.begin() + (long)(data_start + data_len));
        stream.erase(stream.begin(), stream.begin() + (long)total_len);
        return true;
    }

    return false;
}

static bool handle_update_display(const SbepMsg& m, ScreenState& st, Lcd20x4& lcd) {
    // Update Display opcode=$01:
    // data[2]=cc, data[3]=row, data[4]=col, then cc bytes of text.
    if (m.data.size() < 5) return false;

    const uint8_t cc = m.data[2];
    const uint8_t row = m.data[3] & 0x7F;
    if (row > 3) return false;

    if (cc == 0xFF) {
        for (auto& s : st.line) s.clear();
        for (int r = 0; r < 4; ++r) lcd.write_line((uint8_t)r, "");
        return true;
    }
    if (cc == 0) return true;
    if (m.data.size() < 5 + (size_t)cc) return false;

    std::string text;
    text.reserve(cc);
    for (size_t i = 0; i < (size_t)cc; ++i) {
        const uint8_t b = m.data[5 + i];
        const char ch = (b == 0x00) ? ' ' : (char)b;
        if (std::isprint((unsigned char)ch) || ch == ' ') text.push_back(ch);
        else text.push_back(' ');
    }

    std::string shown = text;
    if (row == 2) shown = format_softkeys_5x4(text);
    else {
        replace_caret_with_space(shown);
        rtrim_spaces(shown);
    }

    if ((int)shown.size() > 20) shown.resize(20);
    if (st.line[row] == shown) return true;

    st.line[row] = shown;
    lcd.write_line(row, shown);
    std::printf("LCD row%u: %s\n", row + 1, shown.c_str());
    return true;
}

// Legacy fallback: parse "SBEP-ish" chunks directly from raw stream.
// Some radios/heads expose display text in 1F 00 .. blocks even when strict SBEP
// decoding does not yield opcode=0x01 updates.
static int apply_display_updates_legacy(const uint8_t* data, int len, ScreenState& st, Lcd20x4& lcd) {
    int updates = 0;
    for (int i = 0; i + 10 <= len; ++i) {
        if (data[i] != 0x1F || data[i + 1] != 0x00) continue;

        const uint8_t row = data[i + 6];
        if (row > 3) continue;

        int j = i + 8;
        std::string text;
        text.reserve(40);
        for (; j < len; ++j) {
            const uint8_t c = data[j];
            if (c == 0x00) break;
            if (c == '^' || c == ' ' || (c >= 0x20 && c <= 0x7E)) {
                text.push_back((char)c);
                if ((int)text.size() >= 40) break;
            } else if (!text.empty()) {
                break;
            }
        }

        rtrim_spaces(text);
        if (text.empty()) continue;

        std::string shown = text;
        if (row == 2) shown = format_softkeys_5x4(text);
        else replace_caret_with_space(shown);

        if ((int)shown.size() > 20) shown.resize(20);
        if (st.line[row] == shown) continue;

        st.line[row] = shown;
        lcd.write_line(row, shown);
        std::printf("LCD row%u (legacy): %s\n", row + 1, shown.c_str());
        updates++;
    }
    return updates;
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

    std::vector<uint8_t> payload(4096);
    std::vector<uint8_t> stream;
    stream.reserve(128 * 1024);

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

        std::printf("[CMD] RX frame: %u bytes\n", len);
        stream.insert(stream.end(), payload.begin(), payload.begin() + len);

        int extracted = 0;
        int sbep_updates = 0;
        while (true) {
            SbepMsg m{};
            if (!sbep_extract_next(stream, m)) break;
            extracted++;
            if (m.opcode == 0x01 && handle_update_display(m, st, lcd)) {
                sbep_updates++;
            }
        }

        int legacy_updates = 0;
        if (sbep_updates == 0) {
            legacy_updates = apply_display_updates_legacy(payload.data(), (int)len, st, lcd);
        }

        std::printf("[CMD] SBEP extracted=%d updates=%d legacy=%d stream_buf=%zu\n",
                    extracted, sbep_updates, legacy_updates, stream.size());
    }

    ::close(ls);
    lcd.close_dev();
    std::printf("[CMD] Screen RX stopped\n");
}
