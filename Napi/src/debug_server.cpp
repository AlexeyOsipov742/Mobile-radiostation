#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <atomic>
#include <vector>

static std::atomic<bool> g_run{true};
static void on_sig(int) { g_run = false; }

static int open_uart(const char* dev, int baud = B9600) {
    int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    termios tio{};
    if (tcgetattr(fd, &tio) < 0) {
        ::close(fd);
        return -1;
    }

    cfmakeraw(&tio);
    cfsetispeed(&tio, baud);
    cfsetospeed(&tio, baud);

    // 8N1, без HW flow control
    tio.c_cflag |= (CLOCAL | CREAD | CS8);
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

    // чистый RAW
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0;

    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        ::close(fd);
        return -1;
    }

    // опционально: опустить RTS (иногда полезно на переходниках)
    int status = 0;
    if (ioctl(fd, TIOCMGET, &status) == 0) {
        status &= ~TIOCM_RTS;
        ioctl(fd, TIOCMSET, &status);
    }

    return fd;
}

static std::string find_ttyusb() {
    // пробуем /dev/ttyUSB0..9
    for (int i = 0; i < 10; ++i) {
        char path[64];
        std::snprintf(path, sizeof(path), "/dev/ttyUSB%d", i);
        int fd = open_uart(path);
        if (fd >= 0) {
            ::close(fd);
            return std::string(path);
        }
    }
    return {};
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

static bool send_frame(const char* ip, int port, const uint8_t* payload, uint16_t len) {
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

    // рамка: [len_le][payload]
    uint16_t len_le = htole16(len);
    bool ok = send_all(s, &len_le, sizeof(len_le)) && send_all(s, payload, len);

    ::close(s);
    return ok;
}

static uint16_t read_uart_window(int fd, uint8_t* out, uint16_t cap, int window_ms) {
    uint16_t total = 0;

    const int tick_us = 5000; // 5ms
    int remaining_us = window_ms * 1000;

    while (remaining_us > 0 && total < cap) {
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
                    // просто продолжаем
                } else {
                    break;
                }
            }
        }
        remaining_us -= tick_us;
    }

    return total;
}

int main(int argc, char** argv) {
    std::signal(SIGINT,  on_sig);
    std::signal(SIGTERM, on_sig);

    // Параметры:
    //   debug_server <pi_ip> [port=7777] [tty=/dev/ttyUSB0|auto] [window_ms=300]
    const char* pi_ip = (argc >= 2) ? argv[1] : "192.168.31.XXX";
    int port = (argc >= 3) ? std::atoi(argv[2]) : 7777;
    std::string tty = (argc >= 4) ? std::string(argv[3]) : "auto";
    int window_ms = (argc >= 5) ? std::atoi(argv[4]) : 300;

    if (tty == "auto") {
        tty = find_ttyusb();
        if (tty.empty()) {
            std::fprintf(stderr, "No /dev/ttyUSB* found/openable\n");
            return 1;
        }
    }

    int uart = open_uart(tty.c_str());
    if (uart < 0) {
        std::fprintf(stderr, "Failed to open UART %s: %s\n", tty.c_str(), std::strerror(errno));
        return 1;
    }

    std::printf("DEBUG_SERVER (NaPi): UART RAW -> TCP len+payload\n");
    std::printf("UART: %s, window=%d ms\n", tty.c_str(), window_ms);
    std::printf("Target: %s:%d\n", pi_ip, port);
    std::printf("Press Ctrl+C to stop.\n\n");

    std::vector<uint8_t> buf(4096);

    while (g_run) {
        uint16_t n = read_uart_window(uart, buf.data(), (uint16_t)buf.size(), window_ms);
        if (n > 0) {
            if (!send_frame(pi_ip, port, buf.data(), n)) {
                std::fprintf(stderr, "send_frame failed: %s\n", std::strerror(errno));
            } else {
                std::printf("[UART->NET] sent %u bytes\n", n);
            }
        }
        usleep(20 * 1000);
    }

    ::close(uart);
    std::printf("\nDEBUG_SERVER stopped.\n");
    return 0;
}
