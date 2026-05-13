// uart_buttons_rx.cpp
// NaPi: TCP buttons port -> UART
// SB9600 arbitration is driven by GPIO RTS/CTS (no USB-UART modem lines).
// Also: CRTSCTS must be OFF, write only 5 bytes (no 128 padding).
// We DO NOT read station response here to avoid stealing bytes from UART_FWD thread.

#include "uart_buttons_rx.h"

#include <atomic>
#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <gpiod.h>
#include <netinet/in.h>
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

struct gpio_flow_t {
    gpiod_chip* chip = nullptr;
    gpiod_line* rts = nullptr;
    gpiod_line* cts = nullptr;
    bool rts_active_low = true;
    bool cts_active_low = false;
};

static int logic_to_raw(bool asserted, bool active_low) {
    if (active_low) return asserted ? 0 : 1;
    return asserted ? 1 : 0;
}

static bool raw_to_logic(int raw, bool active_low) {
    if (active_low) return raw == 0;
    return raw != 0;
}

static bool gpio_flow_init(gpio_flow_t& flow,
                           const char* chip_path,
                           unsigned rts_line_offset,
                           unsigned cts_line_offset,
                           bool rts_active_low,
                           bool cts_active_low) {
    flow.rts_active_low = rts_active_low;
    flow.cts_active_low = cts_active_low;

    flow.chip = gpiod_chip_open(chip_path);
    if (!flow.chip) {
        std::fprintf(stderr, "[BTN_RX] gpiod_chip_open(%s) failed: %s\n",
                     chip_path, std::strerror(errno));
        return false;
    }

    flow.rts = gpiod_chip_get_line(flow.chip, rts_line_offset);
    flow.cts = gpiod_chip_get_line(flow.chip, cts_line_offset);
    if (!flow.rts || !flow.cts) {
        std::fprintf(stderr, "[BTN_RX] gpiod_chip_get_line(rts=%u, cts=%u) failed\n",
                     rts_line_offset, cts_line_offset);
        return false;
    }

    // Release bus at startup.
    const int rts_release = logic_to_raw(false, flow.rts_active_low);
    if (gpiod_line_request_output(flow.rts, "sb9600-rts", rts_release) != 0) {
        std::fprintf(stderr, "[BTN_RX] request_output RTS failed: %s\n", std::strerror(errno));
        return false;
    }
    if (gpiod_line_request_input(flow.cts, "sb9600-cts") != 0) {
        std::fprintf(stderr, "[BTN_RX] request_input CTS failed: %s\n", std::strerror(errno));
        return false;
    }

    return true;
}

static void gpio_flow_close(gpio_flow_t& flow) {
    if (flow.rts) {
        // Best-effort: release bus before closing.
        (void)gpiod_line_set_value(flow.rts, logic_to_raw(false, flow.rts_active_low));
        gpiod_line_release(flow.rts);
        flow.rts = nullptr;
    }
    if (flow.cts) {
        gpiod_line_release(flow.cts);
        flow.cts = nullptr;
    }
    if (flow.chip) {
        gpiod_chip_close(flow.chip);
        flow.chip = nullptr;
    }
}

static bool set_rts_asserted(gpio_flow_t& flow, bool asserted) {
    if (!flow.rts) return false;
    const int raw = logic_to_raw(asserted, flow.rts_active_low);
    return gpiod_line_set_value(flow.rts, raw) == 0;
}

static bool get_cts_asserted(gpio_flow_t& flow) {
    if (!flow.cts) return false;
    const int raw = gpiod_line_get_value(flow.cts);
    if (raw < 0) return false;
    return raw_to_logic(raw, flow.cts_active_low);
}

// wait CTS stable N consecutive samples
static bool wait_cts_stable(gpio_flow_t& flow, bool want_high, int stable_count, int timeout_ms) {
    const int step_ms = 5;
    int ok = 0;
    int waited = 0;
    while (waited < timeout_ms) {
        bool cts = get_cts_asserted(flow);
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

static void send_ack_status(int s, bool ok) {
    const uint8_t b = ok ? 0x01 : 0x00;
    (void)::send(s, &b, 1, MSG_NOSIGNAL);
}

void uart_buttons_rx_thread(int port,
                            const char* tty,
                            int baud_bps,
                            const char* gpiochip_path,
                            unsigned rts_line_offset,
                            unsigned cts_line_offset,
                            bool rts_active_low,
                            bool cts_active_low,
                            bool tx_invert) {
    int uart = open_uart_sb9600(tty, baud_bps);
    if (uart < 0) {
        std::fprintf(stderr, "[BTN_RX] open %s failed: %s\n", tty, std::strerror(errno));
        return;
    }

    gpio_flow_t flow{};
    if (!gpio_flow_init(flow, gpiochip_path, rts_line_offset, cts_line_offset, rts_active_low, cts_active_low)) {
        std::fprintf(stderr, "[BTN_RX] GPIO flow init failed, stop\n");
        ::close(uart);
        gpio_flow_close(flow);
        return;
    }

    int ls = ::socket(AF_INET, SOCK_STREAM, 0);
    if (ls < 0) {
        std::fprintf(stderr, "[BTN_RX] socket failed: %s\n", std::strerror(errno));
        gpio_flow_close(flow);
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
        gpio_flow_close(flow);
        ::close(uart);
        return;
    }
    if (::listen(ls, 10) != 0) {
        std::fprintf(stderr, "[BTN_RX] listen failed: %s\n", std::strerror(errno));
        ::close(ls);
        gpio_flow_close(flow);
        ::close(uart);
        return;
    }

    std::fprintf(stdout,
                 "[BTN_RX] TCP :%d -> SB9600 UART %s @%d (expects 5 bytes, writes 5), "
                 "RTS=%s:%u (active_%s), CTS=%s:%u (active_%s), tx_invert=%d\n",
                 port,
                 tty,
                 baud_bps,
                 gpiochip_path,
                 rts_line_offset,
                 rts_active_low ? "LOW" : "HIGH",
                 gpiochip_path,
                 cts_line_offset,
                 cts_active_low ? "LOW" : "HIGH",
                 tx_invert ? 1 : 0);
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
        if (!ok) {
            send_ack_status(c, false);
            ::close(c);
            continue;
        }

        // --- SB9600 transaction (TX only; RX is handled by UART_FWD thread) ---
        bool tx_ok = false;

        // Wait bus free: on your station/adapter idle CTS=0
        if (!wait_cts_stable(flow, /*want_high=*/false, /*stable_count=*/10, /*timeout_ms=*/500)) {
            std::fprintf(stderr, "[BTN_RX] bus busy (CTS!=0), drop cmd: ");
            dump5(cmd);
            std::fprintf(stderr, "\n");
            send_ack_status(c, false);
            ::close(c);
            continue;
        }

        // TAKE BUS:
        // Assert RTS on GPIO.
        if (!set_rts_asserted(flow, true)) {
            std::fprintf(stderr, "[BTN_RX] set RTS active failed: %s\n", std::strerror(errno));
            send_ack_status(c, false);
            ::close(c);
            continue;
        }
        msleep(5);

        // Write exactly 5 bytes
        uint8_t out_cmd[5];
        for (int i = 0; i < 5; ++i) out_cmd[i] = tx_invert ? (uint8_t)(cmd[i] ^ 0xFF) : cmd[i];
        ssize_t w = ::write(uart, out_cmd, 5);
        if (w != 5) {
            std::fprintf(stderr, "[BTN_RX] uart write failed (w=%zd): %s\n", w, std::strerror(errno));
        } else {
            // Ensure bytes pushed
            tcdrain(uart);

            // Small grace period: station may start replying immediately
            (void)wait_cts_stable(flow, /*want_high=*/true, /*stable_count=*/2, /*timeout_ms=*/200);
            msleep(2);

            std::fprintf(stdout, "[BTN_RX] -> UART(5): ");
            dump5(cmd);
            std::fprintf(stdout, "\n");
            std::fflush(stdout);
            tx_ok = true;
        }

        // RELEASE BUS.
        (void)set_rts_asserted(flow, false);

        send_ack_status(c, tx_ok);
        ::close(c);
    }

    ::close(ls);
    gpio_flow_close(flow);
    ::close(uart);
    std::fprintf(stdout, "[BTN_RX] stopped\n");
}
    
