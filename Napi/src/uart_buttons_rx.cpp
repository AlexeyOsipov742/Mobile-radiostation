// uart_buttons_rx.cpp
// NaPi: TCP buttons port -> UART (/dev/ttyUSB0)
// FIXED for SB9600 bus arbitration on USB-UART (RTS inverted on pin):
//   RTS bit=0 -> pin 3.3V (released)
//   RTS bit=1 -> pin 0V   (take bus)
// Also: CRTSCTS must be OFF, write only 5 bytes (no 128 padding).
// We DO NOT read station response here to avoid stealing bytes from UART_FWD thread.

#include "uart_buttons_rx.h"

#include <atomic>
#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static std::atomic<bool> g_btn_run{true};

void uart_buttons_rx_stop() { g_btn_run = false; }

static void msleep(int ms) {
    timespec ts{};
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, nullptr);
}

static int get_modem(int fd) {
    int st = 0;
    if (ioctl(fd, TIOCMGET, &st) < 0) return -1;
    return st;
}

static bool set_line(int fd, int mask, bool on) {
    int st = get_modem(fd);
    if (st < 0) return false;
    if (on) st |= mask; else st &= ~mask;
    return (ioctl(fd, TIOCMSET, &st) == 0);
}

static bool get_cts(int fd) {
    int st = get_modem(fd);
    if (st < 0) return false;
    return (st & TIOCM_CTS) != 0;
}

// wait CTS stable N consecutive samples
static bool wait_cts_stable(int fd, bool want_high, int stable_count, int timeout_ms) {
    const int step_ms = 5;
    int ok = 0;
    int waited = 0;
    while (waited < timeout_ms) {
        bool cts = get_cts(fd);
        if (cts == want_high) ok++; else ok = 0;
        if (ok >= stable_count) return true;
        msleep(step_ms);
        waited += step_ms;
    }
    return false;
}

static void dump5(const uint8_t* b) {
    std::printf("%02X %02X %02X %02X %02X", b[0], b[1], b[2], b[3], b[4]);
}

static int open_uart_sb9600(const char* dev, int baud_bps) {
    int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    termios options{};
    if (tcgetattr(fd, &options) < 0) { ::close(fd); return -1; }

    // raw
    cfmakeraw(&options);

    // speed
    speed_t sp = B9600;
    switch (baud_bps) {
        case 9600:   sp = B9600; break;
        case 19200:  sp = B19200; break;
        case 38400:  sp = B38400; break;
        case 57600:  sp = B57600; break;
        case 115200: sp = B115200; break;
        default:     sp = B9600; break;
    }
    cfsetispeed(&options, sp);
    cfsetospeed(&options, sp);

    // 8N1
    options.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
    options.c_cflag |= CS8;
    options.c_cflag |= CLOCAL | CREAD;

    // IMPORTANT: NO hardware flow control for SB9600
    options.c_cflag &= ~CRTSCTS;

    // non-blocking reads are fine; we don't read here anyway
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &options) < 0) { ::close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);

    // Line policy that doesn't kill the station:
    // DTR=0, RTS RELEASED = bit 0 (pin 3.3V on your adapter)
    set_line(fd, TIOCM_DTR, false);
    set_line(fd, TIOCM_RTS, false);

    return fd;
}

static bool recv_exact(int s, uint8_t* out, size_t n) {
    size_t got = 0;
    while (got < n) {
        ssize_t r = ::recv(s, out + got, n - got, 0);
        if (r == 0) return false; // peer closed
        if (r < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        got += (size_t)r;
    }
    return true;
}

void uart_buttons_rx_thread(int port, const char* tty, int baud_bps) {
    int uart = open_uart_sb9600(tty, baud_bps);
    if (uart < 0) {
        std::fprintf(stderr, "[BTN_RX] open %s failed: %s\n", tty, std::strerror(errno));
        return;
    }

    int ls = ::socket(AF_INET, SOCK_STREAM, 0);
    if (ls < 0) {
        std::fprintf(stderr, "[BTN_RX] socket failed: %s\n", std::strerror(errno));
        ::close(uart);
        return;
    }

    int yes = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons((uint16_t)port);

    if (::bind(ls, (sockaddr*)&a, sizeof(a)) != 0) {
        std::fprintf(stderr, "[BTN_RX] bind :%d failed: %s\n", port, std::strerror(errno));
        ::close(ls);
        ::close(uart);
        return;
    }
    if (::listen(ls, 10) != 0) {
        std::fprintf(stderr, "[BTN_RX] listen failed: %s\n", std::strerror(errno));
        ::close(ls);
        ::close(uart);
        return;
    }

    std::fprintf(stdout, "[BTN_RX] TCP :%d -> SB9600 UART %s @%d (expects 5 bytes, writes 5)\n",
                 port, tty, baud_bps);
    std::fflush(stdout);

    while (g_btn_run.load()) {
        sockaddr_in ca{};
        socklen_t clen = sizeof(ca);
        int c = ::accept(ls, (sockaddr*)&ca, &clen);
        if (c < 0) {
            if (errno == EINTR) continue;
            std::fprintf(stderr, "[BTN_RX] accept failed: %s\n", std::strerror(errno));
            break;
        }

        uint8_t cmd[5];
        bool ok = recv_exact(c, cmd, 5);
        ::close(c);
        if (!ok) continue;

        // --- SB9600 transaction (TX only; RX is handled by UART_FWD thread) ---

        // Wait bus free: on your station/adapter idle CTS=0
        if (!wait_cts_stable(uart, /*want_high=*/false, /*stable_count=*/10, /*timeout_ms=*/500)) {
            std::fprintf(stderr, "[BTN_RX] bus busy (CTS!=0), drop cmd: ");
            dump5(cmd);
            std::fprintf(stderr, "\n");
            continue;
        }

        // TAKE BUS:
        // On your adapter: RTS bit=1 => pin 0V (ACTIVE) => take bus
        set_line(uart, TIOCM_RTS, true);
        msleep(5);

        // Write exactly 5 bytes
        ssize_t w = ::write(uart, cmd, 5);
        if (w != 5) {
            std::fprintf(stderr, "[BTN_RX] uart write failed (w=%zd): %s\n", w, std::strerror(errno));
        } else {
            // Ensure bytes pushed
            tcdrain(uart);

            // Small grace period: station may start replying immediately
            (void)wait_cts_stable(uart, /*want_high=*/true, /*stable_count=*/2, /*timeout_ms=*/200);
            msleep(2);

            std::fprintf(stdout, "[BTN_RX] -> UART(5): ");
            dump5(cmd);
            std::fprintf(stdout, "\n");
            std::fflush(stdout);
        }

        // RELEASE BUS: RTS bit=0 => pin 3.3V
        set_line(uart, TIOCM_RTS, false);
    }

    ::close(ls);
    ::close(uart);
    std::fprintf(stdout, "[BTN_RX] stopped\n");
}
    