// audioTxEth_PI.cpp (NaPi) — MCP3201 -> TCP (mono S16LE @ 12000)
// FIX (только то, что про частоту/тайминг):
//  - Убраны (закомментированы) тяжёлые принты в аудиопетле
//  - Добавлены SCHED_FIFO + mlockall для снижения джиттера
// DEBUG:
//  - Раз в ~2 сек печатает статистику по S16 пакету + raw_u12 min/max/span по пакету
//  - Раз в ~1 сек печатает raw_u12 min/max/span за последнюю секунду
// WAV DUMP:
//  - Пишет то, что реально уходит по сети (S16LE) в "napi_tx.wav" в текущей директории
//  - Важно: close() вызывается ДО usleep(), чтобы заголовок WAV всегда успевал обновиться

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
        std::fprintf(stderr, "[TX] mlockall failed: %s\n", std::strerror(errno));
    }

    // FIFO priority
    sched_param sp{};
    sp.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        std::fprintf(stderr, "[TX] sched_setscheduler(SCHED_FIFO) failed: %s\n", std::strerror(errno));
    }

    // Optional: pin to CPU0
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(0, &set);
    if (sched_setaffinity(0, sizeof(set), &set) != 0) {
        std::fprintf(stderr, "[TX] sched_setaffinity failed: %s\n", std::strerror(errno));
    }
}

// ---------- DEBUG helpers ----------
static void dump_s16_stats_tx(uint32_t pkt_id,
                              const int16_t* s, size_t n, uint32_t fs_hz,
                              uint16_t raw_min_u12, uint16_t raw_max_u12)
{
    if (!s || n == 0) return;

    int16_t mn = s[0], mx = s[0];
    int64_t sum = 0;
    double sumsq = 0.0;
    size_t zeros = 0, clip = 0;

    for (size_t i = 0; i < n; ++i) {
        int16_t v = s[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
        sumsq += (double)v * (double)v;
        if (v == 0) zeros++;
        if (v == INT16_MAX || v == INT16_MIN) clip++;
    }

    double mean = (double)sum / (double)n;
    double rms  = std::sqrt(sumsq / (double)n);
    double pk   = std::max(std::abs((int)mn), std::abs((int)mx));

    char first[256];
    int off = 0;
    off += std::snprintf(first + off, sizeof(first) - (size_t)off, "[");
    size_t show = (n < 10) ? n : 10;
    for (size_t i = 0; i < show; ++i) {
        off += std::snprintf(first + off, sizeof(first) - (size_t)off,
                             "%d%s", (int)s[i], (i + 1 == show) ? "]" : ",");
        if (off >= (int)sizeof(first)) break;
    }

    std::fprintf(stderr,
                 "[TX] pkt=%u n=%zu fs=%u raw_u12[min=%u max=%u span=%d] "
                 "s16[min=%d max=%d mean=%.1f rms=%.1f pk=%.0f zeros=%zu clip=%zu] first=%s\n",
                 pkt_id, n, fs_hz,
                 raw_min_u12, raw_max_u12, (int)raw_max_u12 - (int)raw_min_u12,
                 (int)mn, (int)mx, mean, rms, pk, zeros, clip, first);
}

// ---------- WAV writer ----------
struct WavWriter {
    FILE* f{nullptr};
    uint32_t data_bytes{0};
    uint32_t sample_rate{12000};
    uint16_t channels{1};
    uint16_t bits_per_sample{16};

    static void write_u32_le(FILE* fp, uint32_t v) {
        uint8_t b[4] = {(uint8_t)(v), (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
        fwrite(b, 1, 4, fp);
    }
    static void write_u16_le(FILE* fp, uint16_t v) {
        uint8_t b[2] = {(uint8_t)(v), (uint8_t)(v >> 8)};
        fwrite(b, 1, 2, fp);
    }

    bool open_new(const char* path, uint32_t sr, uint16_t ch=1, uint16_t bps=16) {
        close();
        sample_rate = sr;
        channels = ch;
        bits_per_sample = bps;
        data_bytes = 0;

        f = fopen(path, "wb");
        if (!f) return false;

        static char buf[1 << 20];
        setvbuf(f, buf, _IOFBF, sizeof(buf));

        fwrite("RIFF", 1, 4, f);          write_u32_le(f, 0);
        fwrite("WAVE", 1, 4, f);

        fwrite("fmt ", 1, 4, f);          write_u32_le(f, 16);
        write_u16_le(f, 1);               // PCM
        write_u16_le(f, channels);
        write_u32_le(f, sample_rate);
        uint32_t byte_rate = sample_rate * channels * (bits_per_sample / 8);
        write_u32_le(f, byte_rate);
        uint16_t block_align = channels * (bits_per_sample / 8);
        write_u16_le(f, block_align);
        write_u16_le(f, bits_per_sample);

        fwrite("data", 1, 4, f);          write_u32_le(f, 0);
        return true;
    }

    void write_pcm_s16le(const void* pcm, size_t bytes) {
        if (!f || !pcm || bytes == 0) return;
        fwrite(pcm, 1, bytes, f);
        data_bytes += (uint32_t)bytes;
    }

    void close() {
        if (!f) return;
        uint32_t riff_size = 36u + data_bytes;
        fseek(f, 4, SEEK_SET);   write_u32_le(f, riff_size);
        fseek(f, 40, SEEK_SET);  write_u32_le(f, data_bytes);
        fflush(f);
        fclose(f);
        f = nullptr;
    }
};

} // namespace

void audioTxEth_PI(unsigned char *buffer, std::atomic<bool> &running) {
    (void)buffer;

    if (kFs == 0) { std::fprintf(stderr, "[TX] Invalid fs=%u\n", kFs); return; }

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

    /*WavWriter wav_tx;
    if (!wav_tx.open_new("napi_tx.wav", kFs, 1, 16)) {
        std::perror("fopen napi_tx.wav");
    }
    */
    // meter raw
    uint16_t minv = 4095, maxv = 0;
    uint64_t last_meter_ns = now_ns();

    // eff fs (оставлено, но печать выключена)
    uint64_t eff_last_ns = now_ns();
    uint64_t eff_samples = 0;

    uint32_t pkt_id = 0;

    while (running && gpio_get_ptt_level() == 0) {
        uint64_t now0 = now_ns();
        if (now0 > t_next + period_ns * 4) {
            t_next = now0;
        }

        uint16_t pkt_min_u12 = 4095, pkt_max_u12 = 0;

        for (size_t i = 0; i < kSamplesPerPacket; ++i) {
            t_next += period_ns;
            sleep_until_abs(t_next);

            uint16_t raw = adc_read_u12(spi_fd, tr, rxb, cs);

            if (raw < minv) minv = raw;
            if (raw > maxv) maxv = raw;
            if (raw < pkt_min_u12) pkt_min_u12 = raw;
            if (raw > pkt_max_u12) pkt_max_u12 = raw;

            float x = ((int)raw - 2048) / 2048.0f;
            x = hpf1_run(hpf, x) * kRecordGain;
            if (x > 0.999f) x = 0.999f;
            if (x < -0.999f) x = -0.999f;
            out[i] = (int16_t)lrintf(x * 32767.0f);

            eff_samples++;
        }

        pkt_id++;

        if ((pkt_id % 25u) == 0u) {
            dump_s16_stats_tx(pkt_id, out.data(), out.size(), kFs, pkt_min_u12, pkt_max_u12);
        }

        uint64_t now = now_ns();
        if (now - last_meter_ns > 1000000000ull) {
            std::fprintf(stderr, "[TX] raw_u12 range over last ~1s: min=%u max=%u (span=%d)\n",
                         minv, maxv, (int)maxv - (int)minv);
            minv = 4095; maxv = 0;
            last_meter_ns = now;
        }

        if (now - eff_last_ns > 1000000000ull) {
            eff_samples = 0;
            eff_last_ns = now;
        }

        // WAV dump: то, что реально уходит по сети
        //wav_tx.write_pcm_s16le(out.data(), out.size() * sizeof(int16_t));

        if (!send_all(sockfd, (const uint8_t*)out.data(), out.size() * sizeof(int16_t))) break;
        if (gpio_get_ptt_level() != 0) break;
    }

    cs.close();
    ::close(spi_fd);
    ::close(sockfd);
    gpio_set_activity_led(false);

    // ВАЖНО: сначала закрыть WAV (обновит заголовок), потом уже sleep/retry
    //wav_tx.close();

    usleep(kRetryDelayUsec);
}
