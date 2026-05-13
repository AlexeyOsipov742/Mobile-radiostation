// uart_screen_forward.cpp — SB9600 UART (RAW) -> TCP (len+payload)
//
// Здесь НЕТ никакого парсинга/ASCII. На пульте (Pi) уже SBEP-парсер.

#include "TxRx.h"                   // можно убрать, если не нужен
#include "uart_screen_forward.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cstdlib>
#include <string>
#include <vector>

static std::atomic<bool> g_uart_fwd_run{true};

void uart_screen_forward_stop() {
    g_uart_fwd_run = false;
}

static int open_uart_raw(const char* dev, int baud_bps) {
    int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    termios tio{};
    if (tcgetattr(fd, &tio) < 0) { ::close(fd); return -1; }

    cfmakeraw(&tio);
    speed_t sp = B9600;
    switch (baud_bps) {
        case 9600:   sp = B9600; break;
        case 19200:  sp = B19200; break;
        case 38400:  sp = B38400; break;
        case 57600:  sp = B57600; break;
        case 115200: sp = B115200; break;
        default:     sp = B9600; break;
    }
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    // 8N1, без HW flow control
    tio.c_cflag |= (CLOCAL | CREAD | CS8);
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

    // чистый RAW
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0;

    if (tcsetattr(fd, TCSANOW, &tio) < 0) { ::close(fd); return -1; }

    // опционально: опустить RTS
    int status = 0;
    if (ioctl(fd, TIOCMGET, &status) == 0) {
        status &= ~TIOCM_RTS;
        ioctl(fd, TIOCMSET, &status);
    }
    return fd;
}

static bool send_all(int fd, const void* data, size_t n) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(data);
    while (n > 0) {
        ssize_t w = ::send(fd, p, n, 0);
        if (w < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        p += (size_t)w;
        n -= (size_t)w;
    }
    return true;
}

static bool send_frame_len_payload(const char* ip, int port, const uint8_t* payload, uint16_t len) {
    int s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return false;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);
    if (::inet_pton(AF_INET, ip, &addr.sin_addr) != 1) {
        ::close(s);
        return false;
    }

    if (::connect(s, (sockaddr*)&addr, sizeof(addr)) < 0) {
        ::close(s);
        return false;
    }

    uint16_t len_le = htole16(len);
    bool ok = send_all(s, &len_le, sizeof(len_le)) && send_all(s, payload, len);

    ::close(s);
    return ok;
}

static bool env_bool_or_default(const char* name, bool fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    if (std::strcmp(v, "1") == 0 || std::strcmp(v, "true") == 0 || std::strcmp(v, "TRUE") == 0 ||
        std::strcmp(v, "yes") == 0 || std::strcmp(v, "YES") == 0) {
        return true;
    }
    if (std::strcmp(v, "0") == 0 || std::strcmp(v, "false") == 0 || std::strcmp(v, "FALSE") == 0 ||
        std::strcmp(v, "no") == 0 || std::strcmp(v, "NO") == 0) {
        return false;
    }
    return fallback;
}

static int env_int_or_default(const char* name, int fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    char* end = nullptr;
    long x = std::strtol(v, &end, 10);
    if (!end || *end != '\0') return fallback;
    return (int)x;
}

struct DiagStats {
    size_t ck_ok = 0;
    size_t op1 = 0;
    size_t legacy_1f00 = 0;
    size_t ascii_like = 0;
    size_t zeros = 0;
    size_t ff = 0;
    size_t incomplete = 0;
    size_t invalid = 0;
    size_t bad_ck = 0;
};

static inline uint8_t maybe_inv(uint8_t b, bool inv) {
    return inv ? (uint8_t)(b ^ 0xFF) : b;
}

static bool sbep_checksum_ok_virtual(const uint8_t* p, size_t pos, size_t total, bool inv) {
    if (total < 2) return true;
    uint32_t sum = 0;
    for (size_t i = pos; i + 1 < pos + total; ++i) sum += maybe_inv(p[i], inv);
    const uint8_t expect = (uint8_t)(0xFF - (sum & 0xFF));
    const uint8_t got = maybe_inv(p[pos + total - 1], inv);
    return expect == got;
}

static DiagStats scan_diag_virtual(const uint8_t* p, size_t n, bool inv) {
    DiagStats st{};
    for (size_t i = 0; i < n; ++i) {
        const uint8_t b = maybe_inv(p[i], inv);
        if (b == 0x00) st.zeros++;
        if (b == 0xFF) st.ff++;
        if ((b >= 0x20 && b <= 0x7E) || b == '\r' || b == '\n' || b == '\t') st.ascii_like++;
        if (i + 1 < n && b == 0x1F && maybe_inv(p[i + 1], inv) == 0x00) st.legacy_1f00++;
    }

    for (size_t pos = 0; pos < n; ++pos) {
        const uint8_t b0 = maybe_inv(p[pos], inv);
        const uint8_t msn = (b0 >> 4) & 0x0F;
        const uint8_t lsn = b0 & 0x0F;
        size_t idx = pos + 1;

        uint16_t opcode = 0;
        if (msn == 0x0F) {
            if (idx >= n) { st.incomplete++; continue; }
            opcode = maybe_inv(p[idx], inv);
            idx += 1;
        } else {
            opcode = msn;
        }

        uint32_t follow = 0;
        if (lsn != 0x0F) {
            follow = lsn;
        } else {
            if (idx + 1 >= n) { st.incomplete++; continue; }
            follow = ((uint16_t)maybe_inv(p[idx], inv) << 8) | (uint16_t)maybe_inv(p[idx + 1], inv);
            idx += 2;
        }

        if (follow > 4096) { st.invalid++; continue; }

        const size_t total = (idx - pos) + (size_t)follow;
        if (pos + total > n) { st.incomplete++; continue; }
        if (follow == 0) continue;

        if (!sbep_checksum_ok_virtual(p, pos, total, inv)) { st.bad_ck++; continue; }

        st.ck_ok++;
        if (opcode == 0x01) st.op1++;
    }

    return st;
}

static void print_hex_preview(const char* tag, const uint8_t* p, size_t n, bool inv, int max_bytes) {
    int m = max_bytes;
    if (m < 1) m = 1;
    if ((size_t)m > n) m = (int)n;
    std::fprintf(stdout, "[UART_FWD][DIAG] %s:", tag);
    for (int i = 0; i < m; ++i) {
        std::fprintf(stdout, " %02X", maybe_inv(p[i], inv));
    }
    if ((size_t)m < n) std::fprintf(stdout, " ...");
    std::fprintf(stdout, "\n");
}

// читаем "окном" window_ms, каждые 5мс select/read
static uint16_t read_uart_window(int fd, uint8_t* out, uint16_t cap, int window_ms) {
    uint16_t total = 0;
    const int tick_us = 5000; // 5ms
    int remaining_us = window_ms * 1000;

    while (remaining_us > 0 && total < cap && g_uart_fwd_run.load()) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = tick_us;

        int r = ::select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (r > 0 && FD_ISSET(fd, &rfds)) {
            ssize_t n = ::read(fd, out + total, cap - total);
            if (n > 0) {
                total += (uint16_t)n;
            } else if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    // продолжаем
                } else {
                    break;
                }
            }
        }
        remaining_us -= tick_us;
    }
    return total;
}

void uart_screen_forward_thread(const char* pi_ip,
                                int port,
                                const char* tty,
                                int baud_bps,
                                int window_ms,
                                bool rx_invert) {
    int uart = open_uart_raw(tty, baud_bps);
    if (uart < 0) {
        std::fprintf(stderr, "[UART_FWD] open %s failed: %s\n", tty, std::strerror(errno));
        return;
    }

    std::vector<uint8_t> buf(4096);
    std::vector<uint8_t> txbuf(4096);
    const bool diag = env_bool_or_default("SB9600_DIAG", false);
    int diag_hex = env_int_or_default("SB9600_DIAG_HEX", 24);
    int diag_every = env_int_or_default("SB9600_DIAG_EVERY", 1);
    if (diag_every < 1) diag_every = 1;
    unsigned diag_frame = 0;

    std::fprintf(stdout,
                 "[UART_FWD] %s @%d -> %s:%d (window=%dms, rx_invert=%d)\n",
                 tty, baud_bps, pi_ip, port, window_ms, rx_invert ? 1 : 0);

    while (g_uart_fwd_run.load()) {
        uint16_t n = read_uart_window(uart, buf.data(), (uint16_t)buf.size(), window_ms);
        if (n > 0) {
            if (diag && ((diag_frame++ % (unsigned)diag_every) == 0)) {
                const DiagStats raw = scan_diag_virtual(buf.data(), n, false);
                const DiagStats inv = scan_diag_virtual(buf.data(), n, true);
                const double raw_ascii = n ? (100.0 * (double)raw.ascii_like / (double)n) : 0.0;
                const double inv_ascii = n ? (100.0 * (double)inv.ascii_like / (double)n) : 0.0;

                const char* suggest = "unclear";
                if (raw.ck_ok >= inv.ck_ok * 2 + 3) suggest = "RX_INVERT=0 looks better";
                else if (inv.ck_ok >= raw.ck_ok * 2 + 3) suggest = "RX_INVERT=1 looks better";
                else if (raw.ck_ok == 0 && inv.ck_ok == 0) suggest = "no valid SBEP checksum in either polarity";

                std::fprintf(stdout,
                             "[UART_FWD][DIAG] n=%u raw{ck=%zu op1=%zu 1F00=%zu ascii=%.1f%% z=%zu ff=%zu "
                             "inc=%zu inv=%zu badck=%zu} "
                             "inv{ck=%zu op1=%zu 1F00=%zu ascii=%.1f%% z=%zu ff=%zu inc=%zu inv=%zu badck=%zu} "
                             "=> %s\n",
                             n,
                             raw.ck_ok, raw.op1, raw.legacy_1f00, raw_ascii, raw.zeros, raw.ff,
                             raw.incomplete, raw.invalid, raw.bad_ck,
                             inv.ck_ok, inv.op1, inv.legacy_1f00, inv_ascii, inv.zeros, inv.ff,
                             inv.incomplete, inv.invalid, inv.bad_ck,
                             suggest);
                print_hex_preview("raw", buf.data(), n, false, diag_hex);
                print_hex_preview("inv", buf.data(), n, true, diag_hex);
            }

            const uint8_t* sendp = buf.data();
            if (rx_invert) {
                if (txbuf.size() < n) txbuf.resize(n);
                for (uint16_t i = 0; i < n; ++i) txbuf[i] = (uint8_t)(buf[i] ^ 0xFF);
                sendp = txbuf.data();
            }

            if (!send_frame_len_payload(pi_ip, port, sendp, n)) {
                std::fprintf(stderr, "[UART_FWD] send_frame failed: %s\n", std::strerror(errno));
            } else {
                std::fprintf(stdout, "[UART_FWD] sent %u bytes\n", n);
            }
        }
        usleep(20 * 1000);
    }

    ::close(uart);
    std::fprintf(stdout, "[UART_FWD] stopped\n");
}
