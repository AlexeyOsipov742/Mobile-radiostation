// audioRxEth_PI.cpp (Raspberry Pi) — TCP -> ALSA
// FIX:
//  - recv_exact(): читаем РОВНО BUFFER_SIZE байт на каждый аудиоблок (TCP может дробить)
//  - "probe" не ломает поток: проверяем KN через MSG_PEEK
//  - Debug КАЖДЫЙ пакет: CRC32(всего буфера) + min/max/rms + head/tail samples
//  - ALSA rate_near на 12000
//  - C++ FIX: убран goto через инициализацию (timeout/rx_pkt_id перенесены выше)
//  - JITTER BUFFER: отдельный поток recv + ring buffer + prefill, чтобы убрать underrun

#include "TxRx.h"

#include <alsa/pcm.h>
#include <arpa/inet.h>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <wiringPi.h>
#include <cmath>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

static constexpr unsigned int kAudioSampleRate = 12000;
static constexpr int kJitterPackets  = 8;
static constexpr int kPrefillPackets = 4;

// ---- CRC32 ----
static uint32_t crc32_fast(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (-(int)(crc & 1)));
    }
    return ~crc;
}

static void dump_packet_stats_rx(uint32_t pkt_id, const unsigned char* buf, int nbytes) {
    if (nbytes < 2) {
        std::printf("[RX] pkt=%u nbytes=%d (too short)\n", pkt_id, nbytes);
        return;
    }
    int samples = nbytes / 2;
    const int16_t* s = reinterpret_cast<const int16_t*>(buf);

    int16_t mn = s[0], mx = s[0];
    int64_t sumsq = 0;
    for (int i = 0; i < samples; i++) {
        int16_t v = s[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        int32_t vv = v;
        sumsq += (int64_t)vv * (int64_t)vv;
    }
    double rms = std::sqrt((double)sumsq / (double)samples) / 32768.0;
    uint32_t c = crc32_fast(buf, (size_t)nbytes);

    std::printf("[RX] pkt=%u crc=%08X min=%d max=%d rms=%.4f head=",
                pkt_id, c, (int)mn, (int)mx, rms);

    for (int i = 0; i < 8 && i < samples; i++) std::printf("%d ", (int)s[i]);

    std::printf("tail=");
    if (samples >= 8) {
        for (int i = 8; i > 0; i--) std::printf("%d ", (int)s[samples - i]);
    }
    std::printf("\n");
}

// читаем ровно want байт или false
static bool recv_exact(int fd, unsigned char* dst, int want) {
    int got = 0;
    while (got < want) {
        int n = ::recv(fd, dst + got, want - got, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("recv");
            return false;
        }
        if (n == 0) return false;
        got += n;
    }
    return true;
}

void audioRxEth_PI(unsigned char *buffer) {
    snd_pcm_t *playback_handle = nullptr;
    snd_pcm_hw_params_t *hw_params = nullptr;

    // C++ FIX: эти переменные ДОЛЖНЫ быть объявлены до любых goto fail;
    fd_set readfds;
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    uint32_t rx_pkt_id = 0;

    const char buttons[] = {'K', 'N'};

    int sockfd = -1, newsockfd = -1;
    struct sockaddr_in serv_addr{}, cli_addr{};

    unsigned int sampleRate = kAudioSampleRate;
    unsigned int resample = 1;

    const int alsa_channels = 2;
    snd_pcm_uframes_t local_buffer = BUFFER_SIZE;
    snd_pcm_uframes_t local_periods = PERIODS;

    socklen_t clilen = 0;

    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
        return;
    }
    int optval = 1;
    (void)setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(PORT);

    if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Bind error");
        close(sockfd);
        return;
    }
    listen(sockfd, 5);
    clilen = sizeof(cli_addr);

    int gpio_pin = 19;
    pinMode(gpio_pin, INPUT);
    pullUpDnControl(gpio_pin, PUD_UP);

    if (snd_pcm_open(&playback_handle, "plughw:0,0", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        perror("Cannot open audio device");
        close(sockfd);
        return;
    }

    if (snd_pcm_hw_params_malloc(&hw_params) < 0) {
        perror("Cannot allocate hw params");
        snd_pcm_close(playback_handle);
        close(sockfd);
        return;
    }
    if (snd_pcm_hw_params_any(playback_handle, hw_params) < 0) {
        perror("Cannot configure hw");
        snd_pcm_hw_params_free(hw_params);
        snd_pcm_close(playback_handle);
        close(sockfd);
        return;
    }

    snd_pcm_hw_params_get_buffer_size(hw_params, &local_buffer);
    snd_pcm_hw_params_get_period_size(hw_params, &local_periods, 0);
    std::printf("Buffer size: %lu, Period size: %lu\n", local_buffer, local_periods);

    if (snd_pcm_hw_params_set_format(playback_handle, hw_params, SND_PCM_FORMAT_S16_LE) < 0) {
        perror("Cannot set format");
        goto fail;
    }
    if (snd_pcm_hw_params_set_access(playback_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED) < 0) {
        perror("Cannot set access");
        goto fail;
    }
    if (snd_pcm_hw_params_set_channels(playback_handle, hw_params, alsa_channels) < 0) {
        perror("Cannot set channels");
        goto fail;
    }

    if (snd_pcm_hw_params_set_rate_near(playback_handle, hw_params, &sampleRate, 0) < 0) {
        perror("Cannot set rate near");
        goto fail;
    }
    std::printf("ALSA rate_near result: %u Hz\n", sampleRate);

    if (snd_pcm_hw_params_set_rate_resample(playback_handle, hw_params, resample) < 0) {
        perror("Cannot set resample");
        goto fail;
    }
    if (snd_pcm_hw_params_set_buffer_size_near(playback_handle, hw_params, &local_buffer) < 0) {
        perror("Cannot set buffer size");
        goto fail;
    }
    if (snd_pcm_hw_params_set_period_size_near(playback_handle, hw_params, &local_periods, 0) < 0) {
        perror("Cannot set period size");
        goto fail;
    }

    if (snd_pcm_hw_params(playback_handle, hw_params) < 0) {
        perror("Cannot apply hw params");
        goto fail;
    }
    snd_pcm_hw_params_free(hw_params);
    hw_params = nullptr;

    // mono->stereo
    static int16_t stereo_buf[(BUFFER_SIZE / 2) * 2];

    struct Ring {
        std::vector<std::vector<unsigned char>> buf;
        int r = 0, w = 0, count = 0;
        std::mutex m;
        std::condition_variable cv;
        bool closed = false;

        Ring() : buf(kJitterPackets, std::vector<unsigned char>(BUFFER_SIZE)) {}

        void push_drop_oldest(const unsigned char* src) {
            std::unique_lock<std::mutex> lk(m);
            if (count == kJitterPackets) {
                // drop oldest
                r = (r + 1) % kJitterPackets;
                count--;
            }
            std::memcpy(buf[w].data(), src, BUFFER_SIZE);
            w = (w + 1) % kJitterPackets;
            count++;
            cv.notify_all();
        }

        // wait until at least n packets available or closed
        bool wait_count_at_least(int n) {
            std::unique_lock<std::mutex> lk(m);
            cv.wait(lk, [&]{ return closed || count >= n; });
            return count >= n;
        }

        bool pop(unsigned char* dst) {
            std::unique_lock<std::mutex> lk(m);
            cv.wait(lk, [&]{ return closed || count > 0; });
            if (count == 0) return false;
            std::memcpy(dst, buf[r].data(), BUFFER_SIZE);
            r = (r + 1) % kJitterPackets;
            count--;
            return true;
        }

        void close() {
            std::unique_lock<std::mutex> lk(m);
            closed = true;
            cv.notify_all();
        }
    };

    while (1) {
        FD_ZERO(&readfds);
        FD_SET(sockfd, &readfds);

        timeout.tv_sec = 0;
        timeout.tv_usec = 50'000;

        int sel = select(sockfd + 1, &readfds, NULL, NULL, &timeout);
        if (sel < 0) { perror("select"); break; }

        if (sel == 0) {
            if (digitalRead(gpio_pin) == LOW) break;
            continue;
        }

        newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);
        if (newsockfd < 0) { perror("accept"); continue; }

        // peek 2 bytes to see if it's KN-control message
        unsigned char peek2[2] = {0, 0};
        int pn = recv(newsockfd, peek2, 2, MSG_PEEK);
        if (pn == 2 && peek2[0] == (unsigned char)buttons[0] && peek2[1] == (unsigned char)buttons[1]) {
            if (!recv_exact(newsockfd, buffer, BUFFER_SIZE)) {
                close(newsockfd);
                newsockfd = -1;
                continue;
            }
            memmove(buffer, buffer + 2, BUFFER_SIZE - 2);
            Tx(buffer);
            Rx(buffer);
            TxEth(buffer);
            memset(buffer, 0, BUFFER_SIZE);
            close(newsockfd);
            newsockfd = -1;
            continue;
        }

        std::printf("Client connected\n");
        system("gpio -g mode 20 out");
        system("gpio -g write 20 1");

        if (snd_pcm_prepare(playback_handle) < 0) std::printf("Error preparing\n");

        // ---- Jitter ring + recv thread ----
        Ring ring;
        std::atomic<bool> stop{false};

        std::thread recv_thread([&]{
            std::vector<unsigned char> tmp(BUFFER_SIZE);
            while (!stop.load()) {
                if (!recv_exact(newsockfd, tmp.data(), BUFFER_SIZE)) {
                    break;
                }
                ring.push_drop_oldest(tmp.data());
            }
            ring.close();
        });

        // Prefill before starting playback
        if (!ring.wait_count_at_least(kPrefillPackets)) {
            // closed before prefill
            stop = true;
            if (recv_thread.joinable()) recv_thread.join();
            close(newsockfd);
            newsockfd = -1;
            system("gpio -g write 20 0");
            memset(buffer, 0, BUFFER_SIZE);
            continue;
        }

        // Playback loop: pop from ring
        while (1) {
            if (!ring.pop(buffer)) {
                // ring closed and empty
                break;
            }

            rx_pkt_id++;
            dump_packet_stats_rx(rx_pkt_id, buffer, BUFFER_SIZE);

            const int mono_samples = BUFFER_SIZE / 2;
            const int16_t *mono = reinterpret_cast<const int16_t*>(buffer);

            for (int i = 0; i < mono_samples; i++) {
                stereo_buf[2*i]   = mono[i];
                stereo_buf[2*i+1] = mono[i];
            }

            int err = snd_pcm_writei(playback_handle, stereo_buf, mono_samples);
            if (err < 0) {
                if (err == -EPIPE) {
                    std::fprintf(stderr, "Temporary underrun, retrying...\n");
                    snd_pcm_prepare(playback_handle);
                } else if (err == -EAGAIN) {
                    std::fprintf(stderr, "Temporary unavailable, retrying...\n");
                    continue;
                } else {
                    std::fprintf(stderr, "snd_pcm_writei error: %s\n", snd_strerror(err));
                }
            }
        }

        stop = true;
        ring.close();
        if (recv_thread.joinable()) recv_thread.join();

        std::printf("Connection closed by client\n");
        close(newsockfd);
        newsockfd = -1;
        system("gpio -g write 20 0");
        memset(buffer, 0, BUFFER_SIZE);
    }

    system("gpio -g write 20 0");
    if (playback_handle) {
        snd_pcm_drop(playback_handle);
        snd_pcm_close(playback_handle);
        playback_handle = nullptr;
    }
    if (newsockfd >= 0) close(newsockfd);
    if (sockfd >= 0) close(sockfd);
    return;

fail:
    if (hw_params) snd_pcm_hw_params_free(hw_params);
    if (playback_handle) {
        snd_pcm_close(playback_handle);
        playback_handle = nullptr;
    }
    if (newsockfd >= 0) close(newsockfd);
    if (sockfd >= 0) close(sockfd);
}

