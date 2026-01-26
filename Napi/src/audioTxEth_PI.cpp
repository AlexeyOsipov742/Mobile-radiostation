// audioTxEth_PI.cpp (NaPi) — MCP3201 -> TCP (mono S16LE @ 12000)
// FIX (только то, что про частоту/тайминг):
//  - Убраны (закомментированы) тяжёлые принты в аудиопетле
//  - Добавлены SCHED_FIFO + mlockall для снижения джиттера
// Остальное оставлено как у тебя.

#include "TxRx.h"

#include <atomic>
#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <vector>

// --- NEW: RT scheduling + mlockall ---
#include <sched.h>
#include <sys/mman.h>

#if __has_include(<gpiod.h>)
#include <gpiod.h>
#elif __has_include(<gpiod/gpiod.h>)
#include <gpiod/gpiod.h>
#else
extern "C" {
struct gpiod_chip;
struct gpiod_line;
typedef struct gpiod_chip gpiod_chip;
typedef struct gpiod_line gpiod_line;
gpiod_chip *gpiod_chip_open(const char *path);
void gpiod_chip_close(gpiod_chip *chip);
gpiod_line *gpiod_chip_get_line(gpiod_chip *chip, unsigned int offset);
int gpiod_line_request_output(gpiod_line *line, const char *consumer, int default_val);
void gpiod_line_release(gpiod_line *line);
int gpiod_line_set_value(gpiod_line *line, int value);
}
#endif

namespace {

constexpr uint32_t kFs = static_cast<uint32_t>(NAPI_LOCAL_FS); // 12000
static_assert(BUFFER_SIZE % 2 == 0, "BUFFER_SIZE must be even");
constexpr size_t   kSamplesPerPacket = BUFFER_SIZE / sizeof(int16_t);

constexpr float    kRecordGain   = 1.0f;
constexpr float    kHpfCutoffHz  = 20.0f;
constexpr uint32_t kRetryDelayUsec = 10'000u;
constexpr float    kPiF = 3.14159265358979323846f;

inline uint16_t mcp3201_parse_u12(const uint8_t rx[2]) {
    return static_cast<uint16_t>(((rx[0] & 0x1F) << 7) | ((rx[1] >> 1) & 0x7F));
}

inline uint64_t now_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

inline void ts_from_ns(uint64_t tns, timespec *ts) {
    ts->tv_sec  = static_cast<time_t>(tns / 1000000000ull);
    ts->tv_nsec = static_cast<long>(tns % 1000000000ull);
}

inline void sleep_until_abs(uint64_t t_abs_ns) {
    uint64_t now = now_ns();
    if (now + 50'000ull < t_abs_ns) {
        timespec ts{};
        ts_from_ns(t_abs_ns - 20'000ull, &ts);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    }
    while (now_ns() < t_abs_ns) { /* spin */ }
}

// ---- DC-blocker ----
struct Hpf1 { float R{}, x1{}, y1{}; };

inline void hpf1_init(Hpf1 &h, float fs, float fc) {
    float R = std::exp(-2.0f * kPiF * fc / fs);
    if (R < 0.0f) R = 0.0f;
    if (R > 0.9999f) R = 0.9999f;
    h.R = R; h.x1 = 0.0f; h.y1 = 0.0f;
}
inline float hpf1_run(Hpf1 &h, float x) {
    float y = x - h.x1 + h.R * h.y1;
    h.x1 = x; h.y1 = y;
    return y;
}

// ---- GPIO CS ----
struct GpioCs {
    gpiod_chip *chip{nullptr};
    gpiod_line *line{nullptr};

    bool open(const char *chip_path, unsigned offset, const char *consumer) {
        chip = gpiod_chip_open(chip_path);
        if (!chip) {
            std::fprintf(stderr, "gpiod_chip_open(%s) failed: %s\n", chip_path, std::strerror(errno));
            return false;
        }
        line = gpiod_chip_get_line(chip, offset);
        if (!line) {
            std::fprintf(stderr, "gpiod_chip_get_line(%s,%u) failed: %s\n", chip_path, offset, std::strerror(errno));
            gpiod_chip_close(chip); chip=nullptr;
            return false;
        }
        if (gpiod_line_request_output(line, consumer, 1) != 0) {
            std::fprintf(stderr, "gpiod_line_request_output(%s,%u) failed: %s\n", chip_path, offset, std::strerror(errno));
            gpiod_chip_close(chip); chip=nullptr; line=nullptr;
            return false;
        }
        (void)gpiod_line_set_value(line, 1);
        return true;
    }

    inline void set(int v) { if (line) (void)gpiod_line_set_value(line, v); }

    void close() {
        if (line) { (void)gpiod_line_set_value(line, 1); gpiod_line_release(line); line=nullptr; }
        if (chip) { gpiod_chip_close(chip); chip=nullptr; }
    }
};

int open_spi(const char *dev, uint32_t &hz_inout) {
    int fd = ::open(dev, O_RDWR);
    if (fd < 0) { std::perror("open spidev"); return -1; }

    uint8_t mode = static_cast<uint8_t>(SPI_MODE_0);
    uint8_t bpw  = 8;

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) { std::perror("SPI_IOC_WR_MODE"); ::close(fd); return -1; }
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) == -1) { std::perror("SPI_IOC_WR_BITS_PER_WORD"); ::close(fd); return -1; }

    uint32_t hz = hz_inout ? hz_inout : 1'000'000u;
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) == -1) { std::perror("SPI_IOC_WR_MAX_SPEED_HZ"); ::close(fd); return -1; }

    uint32_t rd = 0;
    if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &rd) == -1) rd = hz;
    hz_inout = (rd ? rd : hz);
    return fd;
}

bool send_all(int sockfd, const uint8_t *data, size_t bytes) {
    size_t sent = 0;
    while (sent < bytes) {
        ssize_t n = ::send(sockfd, data + sent, bytes - sent, 0);
        if (n < 0) { if (errno == EINTR) continue; std::perror("send"); return false; }
        if (n == 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

inline uint16_t adc_read_u12(int spi_fd, spi_ioc_transfer &tr, uint8_t rx[2], GpioCs &cs) {
    cs.set(0);
    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    cs.set(1);
    if (ret < 0) { std::perror("SPI_IOC_MESSAGE (ADC)"); return 2048; }
    return mcp3201_parse_u12(rx);
}

// --- NEW: best-effort realtime setup ---
static void enable_realtime_best_effort() {
    // lock memory to avoid page faults during streaming
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        // не фейлим — просто предупреждаем
        std::fprintf(stderr, "[TX] mlockall failed: %s\n", std::strerror(errno));
    }

    // FIFO priority
    sched_param sp{};
    sp.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        std::fprintf(stderr, "[TX] sched_setscheduler(SCHED_FIFO) failed: %s\n", std::strerror(errno));
    }

    // Optional: pin to CPU0 (часто помогает)
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(0, &set);
    if (sched_setaffinity(0, sizeof(set), &set) != 0) {
        std::fprintf(stderr, "[TX] sched_setaffinity failed: %s\n", std::strerror(errno));
    }
}

} // namespace

void audioTxEth_PI(unsigned char *buffer, std::atomic<bool> &running) {
    (void)buffer;

    if (gpio_get_ptt_level() != 0) return;
    if (kFs == 0) { std::fprintf(stderr, "[TX] Invalid fs=%u\n", kFs); return; }

    // NEW: reduce jitter
    enable_realtime_best_effort();

    gpio_set_activity_led(true);

    int sockfd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) { std::perror("socket"); gpio_set_activity_led(false); return; }
    int yes = 1;
    (void)setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

    sockaddr_in serv{};
    serv.sin_family = AF_INET;
    serv.sin_port   = htons(PORT);
    serv.sin_addr.s_addr = inet_addr(SERVER_IP);

    if (::connect(sockfd, reinterpret_cast<sockaddr *>(&serv), sizeof(serv)) < 0) {
        std::perror("connect");
        ::close(sockfd);
        gpio_set_activity_led(false);
        usleep(kRetryDelayUsec);
        return;
    }

    uint32_t spi_hz = 1'000'000u;
    int spi_fd = open_spi(NAPI_SPI_DEV, spi_hz);
    if (spi_fd < 0) { ::close(sockfd); gpio_set_activity_led(false); usleep(kRetryDelayUsec); return; }

    GpioCs cs;
    if (!cs.open(NAPI_ADC_CS_CHIP, (unsigned)NAPI_ADC_CS_LINE, "napi-adc-cs")) {
        ::close(spi_fd); ::close(sockfd); gpio_set_activity_led(false); usleep(kRetryDelayUsec); return;
    }

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
    hpf1_init(hpf, (float)kFs, kHpfCutoffHz);

    const uint64_t period_ns = (uint64_t)llround(1e9 / (double)kFs);
    uint64_t t_next = now_ns();

    std::vector<int16_t> out(kSamplesPerPacket);

    // meter raw (оставил, но можно тоже выключить)
    uint16_t minv = 4095, maxv = 0;
    uint64_t last_meter_ns = now_ns();

    // eff fs (оставил, но можно тоже выключить)
    uint64_t eff_last_ns = now_ns();
    uint64_t eff_samples = 0;

    uint32_t pkt_id = 0;

    // std::fprintf(stderr, "[TX] start fs=%u, samples/packet=%zu, packet_time=%.3f ms\n",
    //              kFs, kSamplesPerPacket, 1000.0 * (double)kSamplesPerPacket / (double)kFs);

    while (running && gpio_get_ptt_level() == 0) {
        // если мы заметно отстали — не "догоняем", а пересинхронизируемся
        uint64_t now0 = now_ns();
        if (now0 > t_next + period_ns * 4) {
            t_next = now0;
        }

        for (size_t i = 0; i < kSamplesPerPacket; ++i) {
            t_next += period_ns;
            sleep_until_abs(t_next);

            uint16_t raw = adc_read_u12(spi_fd, tr, rxb, cs);

            if (raw < minv) minv = raw;
            if (raw > maxv) maxv = raw;

            float x = ((int)raw - 2048) / 2048.0f;
            x = hpf1_run(hpf, x) * kRecordGain;
            if (x > 0.999f) x = 0.999f;
            if (x < -0.999f) x = -0.999f;
            out[i] = (int16_t)lrintf(x * 32767.0f);

            eff_samples++;
        }

        pkt_id++;

        // ТЯЖЁЛЫЕ ПРИНТЫ — ВЫКЛЮЧЕНО
        // dump_packet_stats_tx(pkt_id, out.data(), out.size());

        // meter раз в 1 сек — можно оставить (лёгкий), но тоже можно выключить
        uint64_t now = now_ns();
        if (now - last_meter_ns > 1000000000ull) {
            // std::fprintf(stderr, "[TX] ADC raw_u12[min=%4u max=%4u]\n", minv, maxv);
            minv = 4095; maxv = 0;
            last_meter_ns = now;
        }

        // eff fs раз в 1 сек — можно оставить, но пока выключим
        if (now - eff_last_ns > 1000000000ull) {
            // double dt = (double)(now - eff_last_ns) / 1e9;
            // double eff = (dt > 0) ? ((double)eff_samples / dt) : 0.0;
            // std::fprintf(stderr, "[TX] EFF_FS=%.1f (target %u)\n", eff, kFs);
            eff_samples = 0;
            eff_last_ns = now;
        }

        if (!send_all(sockfd, (const uint8_t*)out.data(), out.size() * sizeof(int16_t))) break;
        if (gpio_get_ptt_level() != 0) break;
    }

    cs.close();
    ::close(spi_fd);
    ::close(sockfd);
    gpio_set_activity_led(false);
    usleep(kRetryDelayUsec);
}

