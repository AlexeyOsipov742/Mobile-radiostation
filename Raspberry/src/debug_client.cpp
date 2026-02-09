// debug_client.cpp — Pi: TCP(len_le+payload) -> SBEP parse -> LCD 20x4 + terminal
// Usage:
//   ./debug_client 7777
//   ./debug_client 7777 /dev/i2c-1 0x27
//   ./debug_client 7777 --no-lcd
//
// Требования: i2c-dev, права на /dev/i2c-1 (обычно sudo или группа i2c)

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <string>
#include <vector>

// ---- I2C LCD (PCF8574 @ 0x27), как в твоём lcd_i2c.cpp, но под /dev/i2c-1 ----
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

static int g_lcd_fd = -1;

// линии PCF8574
static const uint8_t LCD_RS = 0x01;  // P0
static const uint8_t LCD_EN = 0x04;  // P2
static const uint8_t LCD_BL = 0x08;  // P3 (active high)
static const uint8_t LCD_RW = 0x00;  // RW не используем

static const uint8_t LINE_ADDR_2004[4] = {0x80, 0xC0, 0x94, 0xD4};

static inline bool lcd_wr(uint8_t b) {
    if (g_lcd_fd < 0) return false;
    return (write(g_lcd_fd, &b, 1) == 1);
}
static inline void lcd_pulse(uint8_t out) {
    lcd_wr(out | LCD_EN);
    usleep(1200);
    lcd_wr(out & ~LCD_EN);
    usleep(1200);
}
static inline void lcd_send_nibble(uint8_t nibble, uint8_t flags) {
    uint8_t out = (nibble & 0xF0) | flags;
    lcd_wr(out);
    lcd_pulse(out);
}
static inline void lcd_send_byte(uint8_t val, uint8_t flags) {
    lcd_send_nibble(val & 0xF0,        flags);
    lcd_send_nibble((val << 4) & 0xF0, flags);
}
static inline void lcd_cmd(uint8_t c) {
    uint8_t flags = LCD_BL | LCD_RW; // RS=0
    lcd_send_byte(c, flags);
    if (c == 0x01 || c == 0x02) usleep(2000);
}
static inline void lcd_chr(uint8_t ch) {
    uint8_t flags = LCD_BL | LCD_RS | LCD_RW; // RS=1
    lcd_send_byte(ch, flags);
}
static inline void lcd_set_cursor(uint8_t row, uint8_t col) {
    if (row > 3) row = 0;
    lcd_cmd(LINE_ADDR_2004[row] + col);
}
static void lcd_init_2004() {
    // включим BL
    lcd_wr(LCD_BL);
    usleep(5000);

    // жёсткая инициализация 8-bit → 4-bit
    usleep(15000);
    lcd_send_nibble(0x30, LCD_BL); usleep(5000);
    lcd_send_nibble(0x30, LCD_BL); usleep(5000);
    lcd_send_nibble(0x30, LCD_BL); usleep(5000);
    lcd_send_nibble(0x20, LCD_BL); usleep(5000);

    lcd_cmd(0x28); // 4-bit, 2-line mode (для 20x4 тоже ок на HD44780)
    lcd_cmd(0x08); // display off
    lcd_cmd(0x01); // clear
    usleep(2000);
    lcd_cmd(0x06); // entry mode
    lcd_cmd(0x0C); // display on, cursor off, blink off
}

static bool lcd_open(const char* dev, int addr) {
    g_lcd_fd = open(dev, O_RDWR);
    if (g_lcd_fd < 0) {
        perror("open i2c");
        return false;
    }
    if (ioctl(g_lcd_fd, I2C_SLAVE, addr) < 0) {
        perror("I2C_SLAVE");
        close(g_lcd_fd);
        g_lcd_fd = -1;
        return false;
    }
    lcd_init_2004();
    return true;
}

static void lcd_close() {
    if (g_lcd_fd >= 0) {
        close(g_lcd_fd);
        g_lcd_fd = -1;
    }
}

static void lcd_write_line(uint8_t row, const std::string& s, int width = 20) {
    if (g_lcd_fd < 0) return;

    std::string out = s;
    if ((int)out.size() > width) out.resize(width);
    if ((int)out.size() < width) out.append(width - (int)out.size(), ' ');

    lcd_set_cursor(row, 0);
    for (char c : out) lcd_chr((uint8_t)c);
}

// ---- networking / signals ----
static std::atomic<bool> g_run{true};
static int g_listen_fd = -1;

static void close_fd(int& fd) { if (fd >= 0) { ::close(fd); fd = -1; } }

static void on_sig(int) {
    g_run = false;
    close_fd(g_listen_fd); // разбудить accept()
}

static bool recv_all(int fd, void* data, size_t n) {
    uint8_t* p = (uint8_t*)data;
    while (n) {
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

static void dump_hex(const uint8_t* p, size_t n, size_t maxn = 64) {
    size_t m = std::min(n, maxn);
    for (size_t i = 0; i < m; ++i) {
        printf("%02X ", (unsigned)p[i]);
        if ((i % 32) == 31) printf("\n");
    }
    if ((m % 32) != 0) printf("\n");
    if (n > maxn) printf("... (%zu more bytes)\n", n - maxn);
}

// ---- SBEP robust extraction ----
struct SbepMsg {
    uint16_t opcode = 0;
    std::vector<uint8_t> data;
};

static bool sbep_checksum_ok(const uint8_t* msg, size_t n) {
    if (n < 2) return true;
    uint32_t sum = 0;
    for (size_t i = 0; i + 1 < n; ++i) sum += msg[i];
    uint8_t expect = (uint8_t)(0xFF - (sum & 0xFF));
    return expect == msg[n - 1];
}

static bool sbep_try_at(const std::vector<uint8_t>& stream,
                        size_t pos,
                        size_t& out_total_len,
                        uint16_t& out_opcode,
                        size_t& out_data_start,
                        size_t& out_data_len)
{
    if (pos >= stream.size()) return false;

    uint8_t b0 = stream[pos];
    uint8_t msn = (b0 >> 4) & 0x0F;
    uint8_t lsn = (b0 & 0x0F);

    size_t idx = pos + 1;

    bool ext_opcode = (msn == 0x0F);
    bool ext_size   = (lsn == 0x0F);

    if (!ext_opcode) {
        out_opcode = msn;
    } else {
        if (idx >= stream.size()) return false;
        out_opcode = stream[idx];
        idx += 1;
    }

    uint32_t follow = 0;
    if (!ext_size) {
        follow = lsn;
    } else {
        if (idx + 1 >= stream.size()) return false;
        // ext size: 2 bytes, MSB-first
        follow = ((uint16_t)stream[idx] << 8) | (uint16_t)stream[idx + 1];
        idx += 2;
    }

    if (follow > 4096) return false;

    size_t total = (idx - pos) + (size_t)follow;
    if (stream.size() < pos + total) return false;

    bool has_ck = (follow > 0);
    size_t data_end = pos + total - (has_ck ? 1 : 0);

    out_total_len = total;
    out_data_start = idx;
    out_data_len = (data_end > idx) ? (data_end - idx) : 0;

    if (has_ck) {
        if (!sbep_checksum_ok(&stream[pos], total)) return false;
    }
    return true;
}

static bool sbep_extract_next(std::vector<uint8_t>& stream, SbepMsg& out) {
    const size_t MAX_KEEP  = 64 * 1024;
    const size_t TAIL_KEEP = 8 * 1024;

    if (stream.empty()) return false;

    for (size_t pos = 0; pos < stream.size(); ++pos) {
        size_t total_len = 0;
        uint16_t opcode = 0;
        size_t data_start = 0;
        size_t data_len = 0;

        if (!sbep_try_at(stream, pos, total_len, opcode, data_start, data_len))
            continue;

        out.opcode = opcode;
        out.data.assign(stream.begin() + (long)data_start, stream.begin() + (long)(data_start + data_len));
        stream.erase(stream.begin(), stream.begin() + (long)(pos + total_len));
        return true;
    }

    if (stream.size() > MAX_KEEP) {
        if (stream.size() > TAIL_KEEP) {
            stream.erase(stream.begin(), stream.end() - (long)TAIL_KEEP);
        }
    }
    return false;
}

// ---- screen state ----
static std::string line1, line2, line3;

static void trim(std::string& s) {
    while (!s.empty() && s.front() == ' ') s.erase(s.begin());
    while (!s.empty() && s.back() == ' ') s.pop_back();
}

static std::string prettify_softkeys(std::string s) {
    for (auto& ch : s) if (ch == '^') ch = ' ';
    // схлопнем пробелы чуть-чуть
    std::string out;
    out.reserve(s.size());
    bool prev_space = false;
    for (char c : s) {
        bool sp = (c == ' ');
        if (sp) {
            if (!prev_space) out.push_back(' ');
        } else {
            out.push_back(c);
        }
        prev_space = sp;
    }
    trim(out);
    return out;
}

static void print_screen_terminal() {
    printf("----- SCREEN -----\n");
    printf("1 строка: %s\n", line1.c_str());
    printf("2 строка: %s\n", line2.c_str());
    printf("3 строка: %s\n", line3.c_str());
    printf("------------------\n");
}

static void push_to_lcd_full() {
    if (g_lcd_fd < 0) return;
    lcd_write_line(0, line1);
    lcd_write_line(1, line2);
    lcd_write_line(2, line3);
}

static void push_to_lcd_row(int row) {
    if (g_lcd_fd < 0) return;
    if (row == 0) lcd_write_line(0, line1);
    if (row == 1) lcd_write_line(1, line2);
    if (row == 2) lcd_write_line(2, line3);
}

static bool handle_update_display(const SbepMsg& m) {
    // Update Display opcode=$01:
    // data[2]=cc, data[3]=row, data[4]=col, далее cc байт текста
    if (m.data.size() < 5) return false;

    uint8_t cc  = m.data[2];
    uint8_t row = m.data[3] & 0x7F;
    // uint8_t col = m.data[4] & 0x7F; // пока игнорируем

    if (cc == 0xFF) {
        line1.clear(); line2.clear(); line3.clear();
        push_to_lcd_full();
        print_screen_terminal();
        return true;
    }
    if (cc == 0) return true;
    if (m.data.size() < 5 + (size_t)cc) return false;

    std::string text;
    text.reserve(cc);
    for (size_t i = 0; i < (size_t)cc; ++i) {
        uint8_t b = m.data[5 + i];
        char ch = (b == 0x00) ? ' ' : (char)b;
        if (isprint((unsigned char)ch) || ch == ' ')
            text.push_back(ch);
        else
            text.push_back(' ');
    }

    if (row == 2) text = prettify_softkeys(text);
    else trim(text);

    if (row == 0) line1 = text;
    else if (row == 1) line2 = text;
    else if (row == 2) line3 = text;
    else return false;

    push_to_lcd_row((int)row);
    print_screen_terminal();
    return true;
}

// ---- TCP server socket ----
static int make_server_socket(int port) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket"); return -1; }

    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
#ifdef SO_REUSEPORT
    setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &one, sizeof(one));
#endif

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);

    if (bind(fd, (sockaddr*)&addr, sizeof(addr)) != 0) {
        perror("bind");
        close(fd);
        return -1;
    }
    if (listen(fd, 10) != 0) {
        perror("listen");
        close(fd);
        return -1;
    }
    return fd;
}

int main(int argc, char** argv) {
    int port = 7777;
    bool no_lcd = false;
    const char* i2c_dev = "/dev/i2c-1";
    int i2c_addr = 0x27;

    // Args:
    //   debug_client <port> [i2c_dev] [addr]   OR  debug_client <port> --no-lcd
    if (argc >= 2) {
        if (strcmp(argv[1], "--no-lcd") == 0) {
            no_lcd = true;
        } else {
            port = atoi(argv[1]);
        }
    }
    for (int i = 2; i < argc; ++i) {
        if (strcmp(argv[i], "--no-lcd") == 0) no_lcd = true;
        else if (i2c_dev == std::string("/dev/i2c-1").c_str()) {
            // (не используем этот путь; ниже перезапишем нормально)
        }
    }
    if (argc >= 3 && strcmp(argv[2], "--no-lcd") != 0) i2c_dev = argv[2];
    if (argc >= 4 && strcmp(argv[3], "--no-lcd") != 0) i2c_addr = (int)strtol(argv[3], nullptr, 0);

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    if (!no_lcd) {
        if (lcd_open(i2c_dev, i2c_addr)) {
            // стартовый текст
            lcd_write_line(0, "Station UI Online");
            lcd_write_line(1, "SB9600 -> TCP OK");
            lcd_write_line(2, "Waiting data...");
            lcd_write_line(3, "");
            usleep(50 * 1000);
        } else {
            printf("[LCD] open failed, continue without LCD.\n");
        }
    } else {
        printf("[LCD] disabled by --no-lcd\n");
    }

    g_listen_fd = make_server_socket(port);
    if (g_listen_fd < 0) {
        lcd_close();
        return 1;
    }

    printf("DEBUG_CLIENT (Pi): listening on :%d\n", port);
    printf("LCD: %s addr 0x%02X (%s)\n", i2c_dev, i2c_addr, (g_lcd_fd >= 0 ? "ON" : "OFF"));
    printf("Ctrl+C to stop.\n\n");

    print_screen_terminal();

    std::vector<uint8_t> stream;
    stream.reserve(128 * 1024);

    while (g_run) {
        sockaddr_in cli{};
        socklen_t slen = sizeof(cli);

        int c = ::accept(g_listen_fd, (sockaddr*)&cli, &slen);
        if (c < 0) {
            if (!g_run) break;
            if (errno == EINTR) continue;
            perror("accept");
            continue;
        }

        uint16_t len_le = 0;
        if (!recv_all(c, &len_le, 2)) { close(c); continue; }
        uint16_t len = le16toh(len_le);
        if (len == 0 || len > 65535) { close(c); continue; }

        std::vector<uint8_t> payload(len);
        if (!recv_all(c, payload.data(), payload.size())) { close(c); continue; }
        close(c);

        printf("[NET] got frame: %u bytes\n", (unsigned)len);
        dump_hex(payload.data(), payload.size(), 64);

        stream.insert(stream.end(), payload.begin(), payload.end());

        int extracted = 0;
        int updates = 0;

        while (true) {
            SbepMsg m{};
            if (!sbep_extract_next(stream, m)) break;
            extracted++;
            if (m.opcode == 0x01) {
                if (handle_update_display(m)) updates++;
            }
        }

        if (g_lcd_fd >= 0) {
            char st[32];
            snprintf(st, sizeof(st), "SBEP:%d UPD:%d", extracted, updates);
            lcd_write_line(3, st);
        }

        printf("[SBEP] extracted=%d, update_display=%d, stream_buf=%zu bytes\n\n",
               extracted, updates, stream.size());
        fflush(stdout);
    }

    close_fd(g_listen_fd);
    lcd_close();
    printf("\nDEBUG_CLIENT stopped.\n");
    return 0;
}

