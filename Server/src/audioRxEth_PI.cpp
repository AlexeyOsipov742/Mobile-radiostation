// audioRxEth_PI.cpp (Raspberry Pi) — TCP -> MCP4822 (DAC) + PTT
//
// ЛОГИКА (как в старой версии, но чтение строго по BUFFER_SIZE):
//  - listen()
//  - accept() (с таймаутом, чтобы не зависнуть и успеть увидеть COR)
//  - ПЕРВЫЙ recv_exact(BUFFER_SIZE) (с таймаутом и проверкой COR)
//      *если не получили полный блок -> закрываем клиента, PTT не трогаем*
//  - только после успешного первого блока: PTT=HIGH, старт DAC, проигрываем первый блок
//  - дальше читаем строго по BUFFER_SIZE в цикле (с таймаутом и проверкой COR)
//  - при recv==0/ошибка -> стоп DAC, PTT=LOW ровно один раз, закрыть сокет
//
// COR-логика:
//  - если COR активен (LOW) — не входим в TX
//  - во время сессии, если COR стал активным — выходим из сессии и опускаем PTT
//  - во время ожидания подключения, если COR стал активным — выходим наружу

#include "TxRx.h"
#include "napi_audio_dma_uapi.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <mutex>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace {

static_assert(BUFFER_SIZE % 2 == 0, "BUFFER_SIZE must be even (S16LE)");

#ifndef RPI_LOCAL_FS
#define RPI_LOCAL_FS SAMPLERATE
#endif
constexpr uint32_t kFs = static_cast<uint32_t>(RPI_LOCAL_FS);
// Must match Client TX chunk size for Client -> Server audio path.
constexpr int kNetChunkBytes = 512;
static_assert((kNetChunkBytes % 2) == 0, "kNetChunkBytes must be even (S16LE)");
static_assert(kNetChunkBytes <= BUFFER_SIZE, "kNetChunkBytes must fit shared buffer");
constexpr const char *kDmaDevPath = "/dev/napi_audio_dma0";
constexpr uint32_t kDmaSpiHz = kFs * 16u;
constexpr useconds_t kPttSettleUs = 5'000u;

inline int clampi(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static bool env_enabled(const char *name) {
  const char *v = std::getenv(name);
  return v && *v && std::strcmp(v, "0") != 0;
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
  if (now + 5000ull < t_next) {
    timespec ts{};
    ts_from_ns(t_next - 3000ull, &ts);
    (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
  }
  while ((now = now_ns()) < t_next) { /* spin */ }
}

// ---------- MCP4822 ----------
inline uint16_t mcp4822_word(bool chB, uint16_t u12) {
  return (uint16_t)(((chB ? 1 : 0) << 15) | (1 << 13) | (1 << 12) | (u12 & 0x0FFF));
}

inline uint16_t s16_to_u12(int16_t s, float vol) {
  float x = (float)s / 32768.0f;
  x *= vol;
  if (x > 0.999f) x = 0.999f;
  if (x < -0.999f) x = -0.999f;
  float y = x * 0.5f + 0.5f;
  int v = (int)lrintf(y * 4095.0f);
  v = clampi(v, 0, 4095);
  return (uint16_t)v;
}

// ---------- recv_exact (с таймаутом и возможностью выйти по COR) ----------
static bool recv_exact_stop_on_cor(int fd,
                                   unsigned char *dst,
                                   int want,
                                   int timeout_ms,
                                   uint64_t no_data_timeout_ns) {
  int got = 0;
  uint64_t last_progress_ns = now_ns();
  while (got < want) {
    // если радио начало принимать — выходим немедленно
    if (gpio_get_cor_level() == 0) return false;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    timeval tv{};
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int sel = ::select(fd + 1, &rfds, nullptr, nullptr, &tv);
    if (sel < 0) {
      if (errno == EINTR) continue;
      std::fprintf(stderr, "[RPI-TX] select(recv) error: errno=%d (%s)\n", errno, std::strerror(errno));
      return false;
    }
    if (sel == 0) {
      if (now_ns() - last_progress_ns > no_data_timeout_ns) {
        std::fprintf(stderr, "[RPI-TX] recv watchdog timeout\n");
        return false;
      }
      // короткий таймаут -> снова проверим COR и продолжим
      continue;
    }

    int n = ::recv(fd, dst + got, want - got, 0);
    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
      std::fprintf(stderr, "[RPI-TX] recv error: errno=%d (%s)\n", errno, std::strerror(errno));
      return false;
    }
    if (n == 0) {
      std::fprintf(stderr, "[RPI-TX] peer closed\n");
      return false;
    }
    got += n;
    last_progress_ns = now_ns();
  }
  return true;
}

static bool write_exact_fd(int fd, const void *src, size_t bytes) {
  const auto *p = static_cast<const uint8_t*>(src);
  size_t sent = 0;
  while (sent < bytes) {
    ssize_t n = ::write(fd, p + sent, bytes - sent);
    if (n < 0) {
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
      std::perror("write(dma)");
      return false;
    }
    if (n == 0) return false;
    sent += (size_t)n;
  }
  return true;
}

static void dump_rx_meter(const char *backend,
                          uint32_t chunks,
                          uint32_t bytes,
                          uint32_t max_gap_ms,
                          uint32_t max_write_ms,
                          const int16_t *s,
                          size_t n)
{
  if (!s || n == 0) return;

  int16_t mn = s[0], mx = s[0];
  int64_t sum = 0;
  double sumsq = 0.0;
  uint32_t near_clip = 0;
  uint32_t near_zero = 0;

  for (size_t i = 0; i < n; ++i) {
    int16_t v = s[i];
    mn = std::min(mn, v);
    mx = std::max(mx, v);
    sum += v;
    sumsq += (double)v * (double)v;
    if (v > 32000 || v < -32000) near_clip++;
    if (v > -64 && v < 64) near_zero++;
  }

  double mean = (double)sum / (double)n;
  double rms = std::sqrt(sumsq / (double)n);

  std::fprintf(stderr,
      "[RX-METER] backend=%s chunks=%u bytes=%u max_gap_ms=%u max_write_ms=%u "
      "s16[min=%d max=%d mean=%.1f rms=%.1f clip=%u zero=%u] first8=",
      backend, chunks, bytes, max_gap_ms, max_write_ms,
      (int)mn, (int)mx, mean, rms, near_clip, near_zero);

  size_t show = std::min<size_t>(8, n);
  for (size_t i = 0; i < show; ++i) {
    std::fprintf(stderr, "%s%d", (i == 0 ? "" : ","), (int)s[i]);
  }
  std::fprintf(stderr, "\n");
}

// ---------- ring buffer ----------
class Ring {
public:
  explicit Ring(size_t cap) : buf(cap), cap(cap) {}

  size_t push(const int16_t *data, size_t n) {
    std::unique_lock<std::mutex> lk(m);
    size_t free = cap - size;
    size_t take = (n > free) ? free : n;
    for (size_t i = 0; i < take; i++) buf[(head++) % cap] = data[i];
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

  void reset_stop() {
    std::lock_guard<std::mutex> lk(m);
    stop = false;
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
    : dev(spi_dev), spi_hz(spi_hz), vol(vol), Fs_out(fs_out), q((size_t)fs_out * 1) {}

  bool start() {
    sfd = ::open(dev, O_RDWR);
    if (sfd < 0) { std::perror("open spidev"); return false; }

    uint8_t mode = (uint8_t)SPI_MODE_0;
    uint8_t bpw  = 8;
    if (ioctl(sfd, SPI_IOC_WR_MODE, &mode) == -1) { std::perror("SPI_IOC_WR_MODE"); ::close(sfd); sfd=-1; return false; }
    if (ioctl(sfd, SPI_IOC_WR_BITS_PER_WORD, &bpw) == -1) { std::perror("SPI_IOC_WR_BITS_PER_WORD"); ::close(sfd); sfd=-1; return false; }
    if (ioctl(sfd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_hz) == -1) { std::perror("SPI_IOC_WR_MAX_SPEED_HZ"); ::close(sfd); sfd=-1; return false; }

    q.reset_stop();
    running.store(true);
    need_reset.store(true);
    th = std::thread(&SpiDac::render, this);
    return true;
  }

  void stop() {
    running.store(false);
    q.request_stop();
    if (th.joinable()) th.join();
    write_u12(2048);
    if (sfd >= 0) ::close(sfd);
    sfd = -1;
  }

  void begin_session() {
    q.reset_stop();
    need_reset.store(true, std::memory_order_release);
  }

  void push_frames(const int16_t *data, size_t frames, int channels) {
    if (!frames) return;
    if (channels <= 1) { (void)q.push(data, frames); return; }
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

    (void)ioctl(sfd, SPI_IOC_MESSAGE(1), &tr);
  }

  void render() {
    const uint64_t T = (uint64_t)llround(1e9 / (double)Fs_out);
    uint64_t t_next = now_ns();

    float env = 0.0f;
    const float fadeUp   = 1.0f / (0.010f * (float)Fs_out);
    const float fadeDown = 1.0f / (0.010f * (float)Fs_out);
    // Prime playback for ~40ms to absorb network jitter without adding large latency.
    const size_t kPrimeStartSamples = std::max<size_t>(1, (size_t)((double)Fs_out * 0.040));
    bool primed = false;

    while (running.load()) {
      if (need_reset.exchange(false, std::memory_order_acquire)) {
        q.clear();
        env = 0.0f;
        t_next = now_ns();
        primed = false;
      }

      if (!primed) {
        if (q.available() < kPrimeStartSamples) {
          env = std::max(0.0f, env - fadeDown);
          t_next += T;
          busy_wait_to(t_next);
          write_u12(2048);
          continue;
        }
        primed = true;
      }

      if (q.available() < 1) {
        env = std::max(0.0f, env - fadeDown);
        t_next += T;
        busy_wait_to(t_next);
        write_u12(2048);
        continue;
      }

      int16_t s16{};
      if (!q.pop_one(s16)) break;

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
    std::fprintf(stderr, "[RPI-TX] Invalid fs=%u\n", kFs);
    return;
  }
  constexpr int kRecvPollMs = 20;
  constexpr uint64_t kNoDataTimeoutNs = 2ull * 1000000000ull;
  const bool rx_debug = env_enabled("AUDIO_RX_DEBUG");
  const bool use_dma_dac = env_enabled("AUDIO_RX_USE_DMA");

  // если радио принимает — не включаем TX
  if (gpio_get_cor_level() == 0) return;

  int dma_fd = -1;
  bool dma_active = false;
  if (use_dma_dac) {
    dma_fd = ::open(kDmaDevPath, O_RDWR | O_CLOEXEC);
    if (dma_fd >= 0) {
      napi_audio_dma_config cfg{};
      cfg.api_version = NAPI_AUDIO_DMA_API_VERSION;
      cfg.mode = NAPI_AUDIO_DMA_MODE_DAC;
      cfg.sample_rate_hz = kFs;
      cfg.spi_speed_hz = kDmaSpiHz;
      cfg.flags = 0; // DAC channel A

      if (::ioctl(dma_fd, NAPI_AUDIO_DMA_IOC_CONFIG, &cfg) == 0 &&
          ::ioctl(dma_fd, NAPI_AUDIO_DMA_IOC_START) == 0) {
        dma_active = true;
        std::fprintf(stderr, "[RPI-TX] using kernel DMA DAC backend: %s spi_hz=%u fs=%u\n",
                     kDmaDevPath, kDmaSpiHz, kFs);
      } else {
        std::perror("[RPI-TX] DMA backend setup failed");
        ::close(dma_fd);
        dma_fd = -1;
      }
    } else if (rx_debug) {
      std::fprintf(stderr, "[RPI-TX] DMA backend unavailable: open(%s) failed: %s\n",
                   kDmaDevPath, std::strerror(errno));
    }
  } else if (rx_debug) {
    std::fprintf(stderr, "[RPI-TX] DMA DAC disabled (set AUDIO_RX_USE_DMA=1 to enable)\n");
  }

  // Fallback path: old userspace SPI renderer.
  SpiDac dac(RPI_SPI_DEV, (uint32_t)RPI_SPI_SPEED_HZ, (float)RPI_PLAYBACK_VOL, kFs);
  bool dac_started = false;
  if (!dma_active) {
    if (rx_debug) {
      std::fprintf(stderr, "[RPI-TX] using fallback spidev DAC backend: %s\n", RPI_SPI_DEV);
    }
    // Держим DAC активным во всем RX-режиме:
    // это дает постоянные SPI-такты (как на клиенте), даже когда аудио еще не пришло.
    if (!dac.start()) {
      std::fprintf(stderr, "[RPI-TX] DAC start failed\n");
      return;
    }
    dac_started = true;
  }

  int sockfd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    std::perror("socket");
    if (dac_started) dac.stop();
    if (dma_fd >= 0) {
      (void)::ioctl(dma_fd, NAPI_AUDIO_DMA_IOC_STOP);
      ::close(dma_fd);
    }
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
    if (dac_started) dac.stop();
    if (dma_fd >= 0) {
      (void)::ioctl(dma_fd, NAPI_AUDIO_DMA_IOC_STOP);
      ::close(dma_fd);
    }
    return;
  }
  if (listen(sockfd, 5) < 0) {
    std::perror("listen");
    ::close(sockfd);
    if (dac_started) dac.stop();
    if (dma_fd >= 0) {
      (void)::ioctl(dma_fd, NAPI_AUDIO_DMA_IOC_STOP);
      ::close(dma_fd);
    }
    return;
  }

  // ждём подключения, пока COR не активировался
  while (gpio_get_cor_level() != 0) {
    // accept() с таймаутом: иначе можно навсегда зависнуть в ожидании клиента
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sockfd, &rfds);

    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 100 * 1000; // 100 ms

    int sel = ::select(sockfd + 1, &rfds, nullptr, nullptr, &tv);
    if (sel < 0) {
      if (errno == EINTR) continue;
      std::perror("select(accept)");
      break;
    }
    if (sel == 0) {
      // таймаут -> снова проверим COR
      continue;
    }

    sockaddr_in cli{};
    socklen_t clilen = sizeof(cli);

    int newsockfd = ::accept(sockfd, (sockaddr *)&cli, &clilen);
    if (newsockfd < 0) {
      if (errno == EINTR) continue;
      std::perror("accept");
      continue;
    }

    if (rx_debug) {
      std::fprintf(stderr, "[RX] accepted Client->Server audio connection, backend=%s\n",
                   dma_active ? "dma" : "spidev");
    }

    uint64_t last_chunk_ns = 0;
    uint64_t last_meter_ns = now_ns();
    uint32_t meter_chunks = 0;
    uint32_t meter_bytes = 0;
    uint32_t max_gap_ms = 0;
    uint32_t max_write_ms = 0;
    bool rx_meter_printed_first = false;

    auto note_chunk_for_meter = [&](const unsigned char *data) {
      uint64_t now = now_ns();
      if (last_chunk_ns != 0) {
        uint32_t gap_ms = (uint32_t)((now - last_chunk_ns) / 1000000ull);
        max_gap_ms = std::max(max_gap_ms, gap_ms);
      }
      last_chunk_ns = now;
      meter_chunks++;
      meter_bytes += (uint32_t)kNetChunkBytes;

      if (rx_debug && (!rx_meter_printed_first || (now - last_meter_ns > 1000000000ull))) {
        dump_rx_meter(dma_active ? "dma" : "spidev", meter_chunks, meter_bytes,
                      max_gap_ms, max_write_ms, (const int16_t *)data,
                      (size_t)kNetChunkBytes / sizeof(int16_t));
        rx_meter_printed_first = true;
        meter_chunks = 0;
        meter_bytes = 0;
        max_gap_ms = 0;
        max_write_ms = 0;
        last_meter_ns = now;
      }
    };

    // 1) сначала пробуем получить ПЕРВЫЙ полный блок (PTT не трогаем, пока нет данных)
    if (!recv_exact_stop_on_cor(newsockfd, buffer, kNetChunkBytes, kRecvPollMs, kNoDataTimeoutNs)) {
      shutdown(newsockfd, SHUT_RDWR);
      ::close(newsockfd);
      continue;
    }

    // если вдруг прилетело "KN" — просто считаем это "не аудио" и не поднимаем PTT
    if (buffer[0] == 'K' && buffer[1] == 'N') {
      shutdown(newsockfd, SHUT_RDWR);
      ::close(newsockfd);
      continue;
    }

    note_chunk_for_meter(buffer);

    // 2) поднимаем PTT ТОЛЬКО ТЕПЕРЬ
    gpio_set_ptt(true);
    usleep(kPttSettleUs);

    if (dma_active) {
      uint64_t wr0 = now_ns();
      if (!write_exact_fd(dma_fd, buffer, kNetChunkBytes)) {
        shutdown(newsockfd, SHUT_RDWR);
        ::close(newsockfd);
        gpio_set_ptt(false);
        break;
      }
      max_write_ms = std::max(max_write_ms, (uint32_t)((now_ns() - wr0) / 1000000ull));
    } else {
      // 3) подготовка DAC к новой сессии
      dac.begin_session();

      // проигрываем уже полученный первый блок
      {
        const int channels = 1;
        const int frames = kNetChunkBytes / (2 * channels);
        dac.push_frames((const int16_t *)buffer, (size_t)frames, channels);
      }
    }

    // 4) основной цикл — читаем фиксированный chunk без вечного блока (таймаут + COR)
    while (gpio_get_cor_level() != 0) {
      if (!recv_exact_stop_on_cor(newsockfd, buffer, kNetChunkBytes, kRecvPollMs, kNoDataTimeoutNs)) {
        break;
      }
      if (buffer[0] == 'K' && buffer[1] == 'N') {
        continue;
      }
      note_chunk_for_meter(buffer);
      if (dma_active) {
        uint64_t wr0 = now_ns();
        if (!write_exact_fd(dma_fd, buffer, kNetChunkBytes)) {
          break;
        }
        max_write_ms = std::max(max_write_ms, (uint32_t)((now_ns() - wr0) / 1000000ull));
      } else {
        const int channels = 1;
        const int frames = kNetChunkBytes / (2 * channels);
        dac.push_frames((const int16_t *)buffer, (size_t)frames, channels);
      }
    }

    // 5) завершение сессии: PTT LOW один раз
    shutdown(newsockfd, SHUT_RDWR);
    ::close(newsockfd);
    std::memset(buffer, 0, BUFFER_SIZE);

    gpio_set_ptt(false);

    // если радио начало принимать — выходим наружу
    if (gpio_get_cor_level() == 0) {
      break;
    }
  }

  gpio_set_ptt(false);
  ::close(sockfd);
  if (dac_started) dac.stop();
  if (dma_fd >= 0) {
    (void)::ioctl(dma_fd, NAPI_AUDIO_DMA_IOC_STOP);
    ::close(dma_fd);
  }
}
