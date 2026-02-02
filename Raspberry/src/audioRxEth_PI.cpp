// audioRxEth_PI.cpp (Raspberry Pi) — TCP -> MCP4822 (DAC) + PTT
//
// Этот файл заменяет старую реализацию через ALSA.
// Режим работы:
//   - вызывается из main(), когда COR НЕ активен (digitalRead(RPI_COR_GPIO) == HIGH)
//   - слушает PORT (5678) и принимает TCP соединение от NaPi
//   - при подключении поднимает PTT (RPI_PTT_GPIO = HIGH) и начинает воспроизведение входящего аудио через MCP4822
//   - выходит (возвращается в main), если COR становится активен (LOW) — т.е. радио начало принимать
//
// Протокол аудио:
//   - mono S16LE, BUFFER_SIZE байт на пакет (1024 семпла) @ RPI_LOCAL_FS (по умолчанию 12000)
//
// Управляющие пакеты:
//   - если первые 2 байта при новом подключении == 'K''N', считаем это управляющей посылкой
//     и обрабатываем так же, как в старом коде (Tx/Rx/TxEth), затем закрываем соединение.
//
// DEBUG:
//  - Раз в ~2 сек печатает статистику по принятому S16 блоку + первые 10 семплов + первые 16 байт (hex)

#include "TxRx.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <linux/spi/spidev.h>
#include <mutex>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

#include <sched.h>
#include <sys/mman.h>
#include <sys/stat.h>

namespace {

static_assert(BUFFER_SIZE % 2 == 0, "BUFFER_SIZE must be even (S16LE)");

constexpr uint32_t kFs = static_cast<uint32_t>(RPI_LOCAL_FS);

inline int clampi(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// ---------- DEBUG helpers ----------

struct WavWriter {
    FILE* f{nullptr};
    uint32_t data_bytes{0};
    uint32_t sample_rate{8000};
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
        close(); // close previous if any
        sample_rate = sr;
        channels = ch;
        bits_per_sample = bps;
        data_bytes = 0;

        f = fopen(path, "wb");
        if (!f) return false;

        // big-ish buffer to reduce overhead
        static char buf[1 << 20];
        setvbuf(f, buf, _IOFBF, sizeof(buf));

        // RIFF header placeholder (44 bytes)
        fwrite("RIFF", 1, 4, f);          write_u32_le(f, 0);          // chunk size placeholder
        fwrite("WAVE", 1, 4, f);

        fwrite("fmt ", 1, 4, f);          write_u32_le(f, 16);         // PCM fmt chunk
        write_u16_le(f, 1);               // audio format = PCM
        write_u16_le(f, channels);
        write_u32_le(f, sample_rate);
        uint32_t byte_rate = sample_rate * channels * (bits_per_sample / 8);
        write_u32_le(f, byte_rate);
        uint16_t block_align = channels * (bits_per_sample / 8);
        write_u16_le(f, block_align);
        write_u16_le(f, bits_per_sample);

        fwrite("data", 1, 4, f);          write_u32_le(f, 0);          // data size placeholder
        return true;
    }

    void write_pcm_s16le(const void* pcm, size_t bytes) {
        if (!f || !pcm || bytes == 0) return;
        fwrite(pcm, 1, bytes, f);
        data_bytes += (uint32_t)bytes;
    }

    void close() {
        if (!f) return;

        // finalize header sizes
        uint32_t riff_size = 36u + data_bytes;
        fseek(f, 4, SEEK_SET);   write_u32_le(f, riff_size);
        fseek(f, 40, SEEK_SET);  write_u32_le(f, data_bytes);

        fflush(f);
        fclose(f);
        f = nullptr;
    }
};

static void dump_s16_stats_rx(uint32_t blk_id, const int16_t* s, size_t n, uint32_t fs_hz) {
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

    // первые 10 семплов
    char first[256];
    int off = 0;
    off += std::snprintf(first + off, sizeof(first) - (size_t)off, "[");
    size_t show = (n < 10) ? n : 10;
    for (size_t i = 0; i < show; ++i) {
        off += std::snprintf(first + off, sizeof(first) - (size_t)off,
                             "%d%s", (int)s[i], (i + 1 == show) ? "]" : ",");
        if (off >= (int)sizeof(first)) break;
    }

    // первые 16 байт (hex)
    const uint8_t* b = (const uint8_t*)s;
    char hex[128];
    int ho = 0;
    ho += std::snprintf(hex + ho, sizeof(hex) - (size_t)ho, "[");
    for (int i = 0; i < 16 && (size_t)i < n * 2; ++i) {
        ho += std::snprintf(hex + ho, sizeof(hex) - (size_t)ho,
                            "%02X%s", b[i], (i == 15) ? "]" : " ");
        if (ho >= (int)sizeof(hex)) break;
    }

    std::fprintf(stderr,
                 "[RPI-RX] blk=%u n=%zu fs=%u s16[min=%d max=%d mean=%.1f rms=%.1f pk=%.0f zeros=%zu clip=%zu] first=%s bytes=%s\n",
                 blk_id, n, fs_hz, (int)mn, (int)mx, mean, rms, pk, zeros, clip, first, hex);
}

// ---------- timing ----------
inline uint64_t now_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

inline void ts_from_ns(uint64_t tns, timespec *ts) {
    ts->tv_sec  = (time_t)(tns / 1000000000ull);
    ts->tv_nsec = (long)(tns % 1000000000ull);
}

inline void busy_wait_to(uint64_t t_next) {
    uint64_t now = now_ns();
    if (now + 5'000ull < t_next) {
        timespec ts{};
        ts_from_ns(t_next - 3'000ull, &ts);
        (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    }
    while ((now = now_ns()) < t_next) {
        // spin
    }
}

// ---------- MCP4822 ----------
inline uint16_t mcp4822_word(bool chB, uint16_t u12) {
    // [15]=A/B, [14]=BUF(0), [13]=GA(1=1x), [12]=SHDN(1=active)
    return (uint16_t)(((chB ? 1 : 0) << 15) | (1 << 13) | (1 << 12) | (u12 & 0x0FFF));
}

inline uint16_t s16_to_u12(int16_t s, float vol) {
    float x = (float)s / 32768.0f;
    x *= vol;
    if (x > 0.999f) x = 0.999f;
    if (x < -0.999f) x = -0.999f;
    float y = x * 0.5f + 0.5f;  // [-1..1] -> [0..1]
    int v = (int)lrintf(y * 4095.0f);
    v = clampi(v, 0, 4095);
    return (uint16_t)v;
}

// ---------- recv helpers ----------
// recv_exact с учетом SO_RCVTIMEO: при timeout вернёт false, errno=EAGAIN/EWOULDBLOCK
static bool recv_exact(int fd, unsigned char *dst, int want) {
    int got = 0;
    while (got < want) {
        int n = ::recv(fd, dst + got, want - got, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (n == 0) return false;
        got += n;
    }
    return true;
}

// ---------- ring buffer ----------
class Ring {
public:
    explicit Ring(size_t cap) : buf(cap), cap(cap) {}

    size_t push(const int16_t *data, size_t n) {
        std::unique_lock<std::mutex> lk(m);
        size_t free = cap - size;
        size_t take = (n > free) ? free : n;
        for (size_t i = 0; i < take; i++) {
            buf[(head++) % cap] = data[i];
        }
        size += take;
        lk.unlock();
        cv.notify_one();
        return take;
    }

    bool pop_one(int16_t &out) {
        std::unique_lock<std::mutex> lk(m);
        cv.wait(lk, [&] { return size > 0 || stop; });
        if (size == 0 && stop) return false;
        out = buf[(tail++) % cap];
        size--;
        return true;
    }

    size_t available() {
        std::lock_guard<std::mutex> lk(m);
        return size;
    }

    void clear() {
        std::lock_guard<std::mutex> lk(m);
        head = tail = size = 0;
    }

    void request_stop() {
        std::lock_guard<std::mutex> lk(m);
        stop = true;
        cv.notify_all();
    }

private:
    std::vector<int16_t> buf;
    size_t cap{};
    size_t head{0}, tail{0}, size{0};
    bool stop{false};
    std::mutex m;
    std::condition_variable cv;
};

// ---------- SPI DAC renderer ----------
class SpiDac {
public:
    SpiDac(const char *spi_dev, uint32_t spi_hz, float vol, uint32_t fs_out)
        : dev(spi_dev), spi_hz(spi_hz), vol(vol), Fs_out(fs_out), q((size_t)fs_out * 1) {} // 1 сек буфер

    bool start() {
        sfd = ::open(dev, O_RDWR);
        if (sfd < 0) {
            std::perror("open spidev");
            return false;
        }

        uint8_t mode = (uint8_t)SPI_MODE_0;
        uint8_t bpw  = 8;
        if (ioctl(sfd, SPI_IOC_WR_MODE, &mode) == -1) {
            std::perror("SPI_IOC_WR_MODE");
            ::close(sfd);
            sfd = -1;
            return false;
        }
        if (ioctl(sfd, SPI_IOC_WR_BITS_PER_WORD, &bpw) == -1) {
            std::perror("SPI_IOC_WR_BITS_PER_WORD");
            ::close(sfd);
            sfd = -1;
            return false;
        }
        if (ioctl(sfd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_hz) == -1) {
            std::perror("SPI_IOC_WR_MAX_SPEED_HZ");
            ::close(sfd);
            sfd = -1;
            return false;
        }

        running.store(true);
        need_reset.store(true);
        th = std::thread(&SpiDac::render, this);
        return true;
    }

    void stop() {
        running.store(false);
        q.request_stop();
        if (th.joinable()) th.join();
        // Заглушить выход (перед закрытием SPI)
        write_u12(2048);

        if (sfd >= 0) ::close(sfd);
        sfd = -1;
    }

    void begin_session() {
        need_reset.store(true, std::memory_order_release);
    }

    void push_frames(const int16_t *data, size_t frames, int channels) {
        if (!frames) return;
        if (channels <= 1) {
            (void)q.push(data, frames);
            return;
        }

        // если вдруг пришел стерео — берем левый канал
        tmp.resize(frames);
        for (size_t i = 0; i < frames; i++) tmp[i] = data[i * (size_t)channels];
        (void)q.push(tmp.data(), frames);
    }

private:
    void write_u12(uint16_t u12) {
        if (sfd < 0) return;

        uint16_t w = mcp4822_word(false /*chA*/, u12);
        uint8_t tx[2];
        tx[0] = (uint8_t)(w >> 8);
        tx[1] = (uint8_t)(w & 0xFF);

        spi_ioc_transfer tr{};
        tr.tx_buf = (uintptr_t)tx;
        tr.len = 2;
        tr.speed_hz = spi_hz;
        tr.bits_per_word = 8;
        tr.cs_change = 0;
        tr.delay_usecs = 0;

        // CS (active low) вручную через GPIO
        digitalWrite(RPI_DAC_CS_GPIO, LOW);
        (void)ioctl(sfd, SPI_IOC_MESSAGE(1), &tr);
        digitalWrite(RPI_DAC_CS_GPIO, HIGH);
    }

    void render() {
        // best-effort realtime
        (void)mlockall(MCL_CURRENT | MCL_FUTURE);
        sched_param sp{};
        sp.sched_priority = 60;
        (void)sched_setscheduler(0, SCHED_FIFO, &sp);

        const uint64_t T = (uint64_t)llround(1e9 / (double)Fs_out);
        uint64_t t_next = now_ns();

        float env = 0.0f;
        const float fadeUp   = 1.0f / (0.010f * (float)Fs_out);
        const float fadeDown = 1.0f / (0.010f * (float)Fs_out);

        while (running.load()) {
            if (need_reset.exchange(false, std::memory_order_acquire)) {
                q.clear();
                env = 0.0f;
                t_next = now_ns();
            }

            // underrun -> mid
            if (q.available() < 1) {
                env = std::max(0.0f, env - fadeDown);
                t_next += T;
                busy_wait_to(t_next);
                write_u12(2048);
                continue;
            }

            int16_t s16{};
            if (!q.pop_one(s16)) {
                break;
            }

            // env под запас
            if (q.available() < (size_t)((double)Fs_out * 0.005))
                env = std::max(0.0f, env - fadeDown);
            else
                env = std::min(1.0f, env + fadeUp);

            float y = ((float)s16 / 32768.0f) * env;
            int16_t out = (int16_t)clampi((int)lrintf(y * 32767.0f), -32768, 32767);

            t_next += T;
            busy_wait_to(t_next);
            write_u12(s16_to_u12(out, vol));
        }

        // на выходе заглушим
        write_u12(2048);
    }

    const char *dev;
    uint32_t spi_hz;
    float vol;
    uint32_t Fs_out;

    int sfd{-1};
    std::thread th;
    std::atomic<bool> running{false};

    Ring q;
    std::vector<int16_t> tmp;

    std::atomic<bool> need_reset{false};
};

} // namespace

void audioRxEth_PI(unsigned char *buffer) {
    if (kFs == 0) {
        std::fprintf(stderr, "[RPI-RX] Invalid fs=%u\n", kFs);
        return;
    }

    // Если COR уже активен — не входим в TX-режим
    if (digitalRead(RPI_COR_GPIO) == LOW) {
        return;
    }

    int sockfd = -1;
    sockfd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        std::perror("socket");
        return;
    }

    int optval = 1;
    (void)setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

    sockaddr_in serv{};
    serv.sin_family = AF_INET;
    serv.sin_addr.s_addr = INADDR_ANY;
    serv.sin_port = htons(PORT);

    if (bind(sockfd, (sockaddr *)&serv, sizeof(serv)) < 0) {
        std::perror("bind");
        ::close(sockfd);
        return;
    }
    if (listen(sockfd, 5) < 0) {
        std::perror("listen");
        ::close(sockfd);
        return;
    }

    // accept loop: выходим, если COR станет активным
    while (digitalRead(RPI_COR_GPIO) != LOW) {
        // accept с таймаутом, чтобы регулярно проверять COR
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sockfd, &rfds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 100'000; // 100 ms

        int sel = select(sockfd + 1, &rfds, nullptr, nullptr, &tv);
        if (sel < 0) {
            if (errno == EINTR) continue;
            std::perror("select");
            break;
        }
        if (sel == 0) {
            continue; // timeout
        }

        sockaddr_in cli{};
        socklen_t clilen = sizeof(cli);
        int newsockfd = accept(sockfd, (sockaddr *)&cli, &clilen);
        if (newsockfd < 0) {
            if (errno == EINTR) continue;
            std::perror("accept");
            continue;
        }

        // Peek 2 bytes: управляющий пакет "KN"?
        unsigned char peek2[2] = {0, 0};
        int pn = recv(newsockfd, peek2, 2, MSG_PEEK);
        if (pn == 2 && peek2[0] == (unsigned char)'K' && peek2[1] == (unsigned char)'N') {
            if (recv_exact(newsockfd, buffer, BUFFER_SIZE)) {
                std::memmove(buffer, buffer + 2, BUFFER_SIZE - 2);
                Tx(buffer);
                Rx(buffer);
                TxEth(buffer);
                std::memset(buffer, 0, BUFFER_SIZE);
            }
            ::close(newsockfd);
            continue;
        }

        // --- Audio session ---
        digitalWrite(RPI_PTT_GPIO, HIGH);
        usleep(50'000);

        // recv timeout, чтобы не блокировать проверку COR
        {
            timeval rcvto{};
            rcvto.tv_sec = 0;
            rcvto.tv_usec = 20'000; // 20ms
            (void)setsockopt(newsockfd, SOL_SOCKET, SO_RCVTIMEO, &rcvto, sizeof(rcvto));
        }

        SpiDac dac(RPI_SPI_DEV, (uint32_t)RPI_SPI_SPEED_HZ, (float)RPI_PLAYBACK_VOL, kFs);
        if (!dac.start()) {
            std::fprintf(stderr, "[RPI-RX] DAC start failed\n");
            digitalWrite(RPI_PTT_GPIO, LOW);
            ::close(newsockfd);
            continue;
        }
        dac.begin_session();
//WavWriter wav_rx;
//wav_rx.open_new("rpi_rx.wav", kFs, 1, 16);	
        uint32_t blk_id = 0;

        // RX loop
        while (digitalRead(RPI_COR_GPIO) != LOW) {
            if (!recv_exact(newsockfd, buffer, BUFFER_SIZE)) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                break;
            }

            if (buffer[0] == 'K' && buffer[1] == 'N') {
                continue;
            }
	    //wav_rx.write_pcm_s16le(buffer, BUFFER_SIZE);
            blk_id++;
            if ((blk_id % 25u) == 0u) { // ~2.1 сек при 12кГц и 1024 семпла
                dump_s16_stats_rx(blk_id, (const int16_t*)buffer, (size_t)(BUFFER_SIZE / 2), kFs);
            }

            const int channels = 1;
            const int frames = BUFFER_SIZE / (2 * channels);
            dac.push_frames((const int16_t *)buffer, (size_t)frames, channels);
        }
	//wav_rx.close();	
        dac.stop();
        ::close(newsockfd);
        std::memset(buffer, 0, BUFFER_SIZE);

        digitalWrite(RPI_PTT_GPIO, LOW);

        if (digitalRead(RPI_COR_GPIO) == LOW) {
            break;
        }
    }

    digitalWrite(RPI_PTT_GPIO, LOW);
    ::close(sockfd);
}

