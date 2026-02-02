// audioTxEth_PI.cpp (Raspberry Pi) — MCP3201 (ADC) -> TCP (mono S16LE @ RPI_LOCAL_FS)
//
// Этот файл заменяет старую реализацию через ALSA.
// Режим работы:
//   - вызывается из main(), когда COR активен (digitalRead(RPI_COR_GPIO) == LOW)
//   - читает звук с MCP3201 (SPI) с частотой RPI_LOCAL_FS
//   - отправляет пакеты по BUFFER_SIZE байт на NaPi (SERVER_IP:PORT)
//   - выходит, когда COR становится неактивным

#include "TxRx.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <linux/spi/spidev.h>
#include <netinet/tcp.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace {

static_assert(BUFFER_SIZE % 2 == 0, "BUFFER_SIZE must be even (S16LE)");

constexpr uint32_t kFs = static_cast<uint32_t>(RPI_LOCAL_FS);
constexpr size_t   kSamplesPerPacket = BUFFER_SIZE / sizeof(int16_t);

// ---- timing ----
inline uint64_t now_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

inline void ts_from_ns(uint64_t tns, timespec *ts) {
    ts->tv_sec  = static_cast<time_t>(tns / 1000000000ull);
    ts->tv_nsec = static_cast<long>(tns % 1000000000ull);
}

// Гибрид: сначала nanosleep (TIMER_ABSTIME), потом короткий spin.
inline void sleep_until_abs(uint64_t t_abs_ns) {
    uint64_t now = now_ns();
    if (now + 50'000ull < t_abs_ns) {
        timespec ts{};
        ts_from_ns(t_abs_ns - 20'000ull, &ts);
        (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    }
    while (now_ns() < t_abs_ns) {
        // spin
    }
}

// ---- DC blocker (1st order HPF) ----
constexpr float kPiF = 3.14159265358979323846f;
struct Hpf1 {
    float R{};
    float x1{};
    float y1{};
};

inline void hpf1_init(Hpf1 &h, float fs, float fc) {
    float R = std::exp(-2.0f * kPiF * fc / fs);
    if (R < 0.0f) R = 0.0f;
    if (R > 0.9999f) R = 0.9999f;
    h.R = R;
    h.x1 = 0.0f;
    h.y1 = 0.0f;
}

inline float hpf1_run(Hpf1 &h, float x) {
    float y = x - h.x1 + h.R * h.y1;
    h.x1 = x;
    h.y1 = y;
    return y;
}

// ---- MCP3201 ----
inline uint16_t mcp3201_parse_u12(const uint8_t rx[2]) {
    // upper5 = rx0[4:0] = B11..B7
    // lower7 = rx1[7:1] = B6..B0
    return static_cast<uint16_t>(((rx[0] & 0x1F) << 7) | ((rx[1] >> 1) & 0x7F));
}

int open_spi(const char *dev, uint32_t &hz_inout) {
    int fd = ::open(dev, O_RDWR);
    if (fd < 0) {
        std::perror("open spidev");
        return -1;
    }

    uint8_t mode = static_cast<uint8_t>(SPI_MODE_0);
    uint8_t bpw  = 8;

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) {
        std::perror("SPI_IOC_WR_MODE");
        ::close(fd);
        return -1;
    }
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) == -1) {
        std::perror("SPI_IOC_WR_BITS_PER_WORD");
        ::close(fd);
        return -1;
    }

    uint32_t hz = hz_inout ? hz_inout : 1'000'000u;
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) == -1) {
        std::perror("SPI_IOC_WR_MAX_SPEED_HZ");
        ::close(fd);
        return -1;
    }
    uint32_t rd = 0;
    if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &rd) == -1) rd = hz;
    hz_inout = (rd ? rd : hz);
    return fd;
}

bool send_all(int sockfd, const uint8_t *data, size_t bytes) {
    size_t sent = 0;
    while (sent < bytes) {
        ssize_t n = ::send(sockfd, data + sent, bytes - sent, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            std::perror("send");
            return false;
        }
        if (n == 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

inline uint16_t adc_read_u12(int spi_fd, spi_ioc_transfer &tr, uint8_t rx[2]) {
    // CS (active low) вручную через GPIO
    digitalWrite(RPI_ADC_CS_GPIO, LOW);
    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    digitalWrite(RPI_ADC_CS_GPIO, HIGH);
    if (ret < 0) {
        std::perror("SPI_IOC_MESSAGE (ADC)");
        return 2048;
    }
    return mcp3201_parse_u12(rx);
}

// best-effort realtime: не критично, но может уменьшить джиттер
void enable_realtime_best_effort() {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::fprintf(stderr, "[RPI-TX] mlockall failed: %s\n", std::strerror(errno));
    }

    sched_param sp{};
    sp.sched_priority = 70;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        std::fprintf(stderr, "[RPI-TX] sched_setscheduler(SCHED_FIFO) failed: %s\n", std::strerror(errno));
    }
}

} // namespace

void audioTxEth_PI(unsigned char *buffer) {
    (void)buffer;

    if (kFs == 0) {
        std::fprintf(stderr, "[RPI-TX] Invalid fs=%u\n", kFs);
        return;
    }

    // Если COR уже не активен — сразу выходим
    if (digitalRead(RPI_COR_GPIO) != LOW) {
        return;
    }

    enable_realtime_best_effort();

    // --- TCP connect to NaPi ---
    int sockfd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        std::perror("socket");
        return;
    }

    int yes = 1;
    (void)setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

    sockaddr_in serv{};
    serv.sin_family = AF_INET;
    serv.sin_port   = htons(PORT);
    serv.sin_addr.s_addr = inet_addr(SERVER_IP);

    if (::connect(sockfd, reinterpret_cast<sockaddr *>(&serv), sizeof(serv)) < 0) {
        // NaPi может быть в режиме TX и не слушать — это нормально
        std::perror("connect");
        ::close(sockfd);
        usleep(10'000);
        return;
    }

    // --- SPI open ---
    uint32_t spi_hz = static_cast<uint32_t>(RPI_SPI_SPEED_HZ);
    int spi_fd = open_spi(RPI_SPI_DEV, spi_hz);
    if (spi_fd < 0) {
        ::close(sockfd);
        usleep(10'000);
        return;
    }

    // Подготовим spi transfer (2 bytes)
    uint8_t txb[2] = {0, 0};
    uint8_t rxb[2] = {0, 0};
    spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<uintptr_t>(txb);
    tr.rx_buf = reinterpret_cast<uintptr_t>(rxb);
    tr.len = 2;
    tr.speed_hz = spi_hz;
    tr.bits_per_word = 8;
    tr.cs_change = 0;

    Hpf1 hpf{};
    hpf1_init(hpf, (float)kFs, (float)RPI_HPF_CUTOFF_HZ);

    const uint64_t period_ns = (uint64_t)llround(1e9 / (double)kFs);
    uint64_t t_next = now_ns();

    std::vector<int16_t> out(kSamplesPerPacket);

    while (digitalRead(RPI_COR_GPIO) == LOW) {
        // если сильно отстали — пересинхронизация
        uint64_t now0 = now_ns();
        if (now0 > t_next + period_ns * 4) {
            t_next = now0;
        }

        for (size_t i = 0; i < kSamplesPerPacket; ++i) {
            t_next += period_ns;
            sleep_until_abs(t_next);

            uint16_t raw = adc_read_u12(spi_fd, tr, rxb);

            float x = ((int)raw - 2048) / 2048.0f;
            x = hpf1_run(hpf, x) * (float)RPI_RECORD_GAIN;
            if (x > 0.999f) x = 0.999f;
            if (x < -0.999f) x = -0.999f;
            out[i] = (int16_t)lrintf(x * 32767.0f);
        }

        if (!send_all(sockfd, reinterpret_cast<const uint8_t *>(out.data()), out.size() * sizeof(int16_t))) {
            break;
        }
    }

    // cleanup
    digitalWrite(RPI_ADC_CS_GPIO, HIGH);
    ::close(spi_fd);
    ::close(sockfd);
    usleep(10'000);
}
