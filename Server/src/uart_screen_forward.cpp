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
#include <string>
#include <vector>

static std::atomic<bool> g_uart_fwd_run{true};

void uart_screen_forward_stop() {
    g_uart_fwd_run = false;
}

static int open_uart_raw_9600(const char* dev) {
    int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    termios tio{};
    if (tcgetattr(fd, &tio) < 0) { ::close(fd); return -1; }

    cfmakeraw(&tio);
    cfsetispeed(&tio, B9600);
    cfsetospeed(&tio, B9600);

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

void uart_screen_forward_thread(const char* pi_ip, int port, const char* tty, int /*baud_bps*/, int window_ms) {
    int uart = open_uart_raw_9600(tty);
    if (uart < 0) {
        std::fprintf(stderr, "[UART_FWD] open %s failed: %s\n", tty, std::strerror(errno));
        return;
    }

    std::vector<uint8_t> buf(4096);

    std::fprintf(stdout, "[UART_FWD] %s @9600 -> %s:%d (window=%dms)\n", tty, pi_ip, port, window_ms);

    while (g_uart_fwd_run.load()) {
        uint16_t n = read_uart_window(uart, buf.data(), (uint16_t)buf.size(), window_ms);
        if (n > 0) {
            if (!send_frame_len_payload(pi_ip, port, buf.data(), n)) {
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

