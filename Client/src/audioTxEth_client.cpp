// audioTxEth_client.cpp (NaPi) — MCP3201 -> TCP (mono S16LE @ NAPI_LOCAL_FS)
// FIX: debounce отпускания PTT внутри TX, чтобы глитчи не рвали TCP-сессию.

#include "TxRx.h"
#include "napi_audio_dma_uapi.h"

#include <algorithm>
#include <atomic>
#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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

#include <sched.h>
#include <sys/mman.h>

namespace {

constexpr uint32_t kFs = static_cast<uint32_t>(NAPI_LOCAL_FS);
static_assert(BUFFER_SIZE % 2 == 0, "BUFFER_SIZE must be even");
// Dedicated TX chunk for live audio path Client -> Server:
// 512 bytes = 256 samples @16-bit mono => 32ms at 8kHz.
constexpr size_t kTxChunkBytes = 512;
static_assert((kTxChunkBytes % sizeof(int16_t)) == 0, "kTxChunkBytes must be aligned to int16");
static_assert(kTxChunkBytes <= BUFFER_SIZE, "kTxChunkBytes must fit shared buffer");
constexpr size_t kSamplesPerPacket = kTxChunkBytes / sizeof(int16_t);

constexpr float    kRecordGain   = 1.0f;
constexpr float    kHpfCutoffHz  = 20.0f;
constexpr uint32_t kRetryDelayUsec = 10'000u;
constexpr float    kPiF = 3.14159265358979323846f;
constexpr bool     kTxDebugLog = false;
constexpr const char *kDmaDevPath = "/dev/napi_audio_dma0";
constexpr uint32_t kDmaSpiHz = 1'000'000u;

// отпускание PTT считаем реальным только если держится HIGH >= debounce
constexpr uint64_t kPttReleaseDebounceNs = 40ull * 1000000ull; // 40ms

inline uint16_t mcp3201_parse_u12(const uint8_t rx[2]) {
  return static_cast<uint16_t>(((rx[0] & 0x1F) << 7) | ((rx[1] >> 1) & 0x7F));
}

inline int clamp_int(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
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

ssize_t read_exact_fd(int fd, void *dst, size_t bytes) {
  auto *p = static_cast<uint8_t*>(dst);
  size_t got = 0;
  while (got < bytes) {
    ssize_t n = ::read(fd, p + got, bytes - got);
    if (n < 0) {
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
      std::perror("read(dma)");
      return -1;
    }
    if (n == 0) return -1;
    got += static_cast<size_t>(n);
  }
  return static_cast<ssize_t>(got);
}

inline uint16_t adc_read_u12(int spi_fd, spi_ioc_transfer &tr, uint8_t rx[2]) {
  int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
  if (ret < 0) { std::perror("SPI_IOC_MESSAGE (ADC)"); return 2048; }
  return mcp3201_parse_u12(rx);
}

static void enable_realtime_best_effort() {
  static bool warned_sched = false;
  static bool warned_mlock = false;
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    if (!warned_mlock) {
      std::fprintf(stderr, "[TX] mlockall failed: %s\n", std::strerror(errno));
      warned_mlock = true;
    }
  }
  sched_param sp{};
  sp.sched_priority = 80;
  if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
    if (!warned_sched) {
      std::fprintf(stderr, "[TX] sched_setscheduler(SCHED_FIFO) failed: %s\n", std::strerror(errno));
      warned_sched = true;
    }
  }
}

static void dump_s16_stats_tx(uint32_t pkt_id,
                              const int16_t* s, size_t n, uint32_t fs_hz,
                              uint16_t raw_min_u12, uint16_t raw_max_u12)
{
  if (!s || n == 0) return;
  int16_t mn = s[0], mx = s[0];
  int64_t sum = 0;
  double sumsq = 0.0;
  for (size_t i = 0; i < n; ++i) {
    int16_t v = s[i];
    mn = std::min(mn, v);
    mx = std::max(mx, v);
    sum += v;
    sumsq += (double)v * (double)v;
  }
  double mean = (double)sum / (double)n;
  double rms  = std::sqrt(sumsq / (double)n);
  std::fprintf(stderr,
               "[TX] pkt=%u n=%zu fs=%u raw_u12[min=%u max=%u span=%d] s16[min=%d max=%d mean=%.1f rms=%.1f]\n",
               pkt_id, n, fs_hz,
               raw_min_u12, raw_max_u12, (int)raw_max_u12 - (int)raw_min_u12,
               (int)mn, (int)mx, mean, rms);
}

static bool env_enabled(const char *name) {
  const char *v = std::getenv(name);
  return v && *v && std::strcmp(v, "0") != 0;
}

static void dump_tx_meter(const char *backend,
                          uint32_t pkt_id,
                          uint32_t packets,
                          uint32_t bytes,
                          const int16_t *s,
                          size_t n,
                          uint16_t raw_min_u12,
                          uint16_t raw_max_u12)
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
  uint32_t est_fs = (uint32_t)(packets * (uint32_t)n);

  std::fprintf(stderr,
      "[TX-METER] backend=%s pkt=%u rate_pkts=%u est_fs=%u rate_bytes=%u raw_u12[min=%u max=%u span=%d] "
      "s16[min=%d max=%d mean=%.1f rms=%.1f clip=%u zero=%u] first8=",
      backend, pkt_id, packets, est_fs, bytes, raw_min_u12, raw_max_u12,
      (int)raw_max_u12 - (int)raw_min_u12,
      (int)mn, (int)mx, mean, rms, near_clip, near_zero);

  size_t show = std::min<size_t>(8, n);
  for (size_t i = 0; i < show; ++i) {
    std::fprintf(stderr, "%s%d", (i == 0 ? "" : ","), (int)s[i]);
  }
  std::fprintf(stderr, "\n");
}

} // namespace

void audioTxEth_client(unsigned char *buffer, std::atomic<bool> &running) {
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

  int dma_fd = -1;
  bool dma_active = false;
  const bool tx_debug = kTxDebugLog || env_enabled("AUDIO_TX_DEBUG");
  const bool use_dma_adc = env_enabled("AUDIO_TX_USE_DMA");

  if (use_dma_adc) {
    dma_fd = ::open(kDmaDevPath, O_RDWR | O_CLOEXEC);
    if (dma_fd >= 0) {
      napi_audio_dma_config cfg{};
      cfg.api_version = NAPI_AUDIO_DMA_API_VERSION;
      cfg.mode = NAPI_AUDIO_DMA_MODE_ADC;
      cfg.sample_rate_hz = kFs;
      cfg.spi_speed_hz = kDmaSpiHz;
      cfg.flags = NAPI_AUDIO_DMA_F_ADC_PARSE_MCP3201;

      if (::ioctl(dma_fd, NAPI_AUDIO_DMA_IOC_CONFIG, &cfg) == 0 &&
          ::ioctl(dma_fd, NAPI_AUDIO_DMA_IOC_START) == 0) {
        dma_active = true;
        std::fprintf(stderr, "[TX] using kernel DMA ADC backend: %s\n", kDmaDevPath);
      } else {
        std::perror("[TX] DMA backend setup failed");
        ::close(dma_fd);
        dma_fd = -1;
      }
    } else if (tx_debug) {
      std::fprintf(stderr, "[TX] DMA backend unavailable: open(%s) failed: %s\n",
                   kDmaDevPath, std::strerror(errno));
    }
  } else if (tx_debug) {
    std::fprintf(stderr, "[TX] DMA ADC disabled (set AUDIO_TX_USE_DMA=1 to enable)\n");
  }

  uint32_t spi_hz = 1'000'000u;
  int spi_fd = -1;
  uint8_t txb[2] = {0, 0};
  uint8_t rxb[2] = {0, 0};
  spi_ioc_transfer tr{};

  if (!dma_active) {
    if (tx_debug) {
      std::fprintf(stderr, "[TX] using fallback spidev ADC backend: %s\n", NAPI_SPI_DEV);
    }
    spi_fd = open_spi(NAPI_SPI_DEV, spi_hz);
    if (spi_fd < 0) { ::close(sockfd); gpio_set_activity_led(false); usleep(kRetryDelayUsec); return; }

    tr.tx_buf = reinterpret_cast<uintptr_t>(txb);
    tr.rx_buf = reinterpret_cast<uintptr_t>(rxb);
    tr.len = 2;
    tr.speed_hz = spi_hz;
    tr.bits_per_word = 8;
    tr.cs_change = 0;
  }

  Hpf1 hpf{};
  hpf1_init(hpf, (float)kFs, kHpfCutoffHz);
  bool hpf_primed = false;

  const uint64_t period_ns = (uint64_t)llround(1e9 / (double)kFs);
  const uint64_t chunk_period_ns = period_ns * (uint64_t)kSamplesPerPacket;
  uint64_t t_next = now_ns();

  if (tx_debug && dma_active) {
    std::fprintf(stderr, "[TX] DMA kernel-paced ADC: spi_hz=%u fs=%u chunk_samples=%zu chunk_ms=%.1f\n",
                 kDmaSpiHz, kFs, kSamplesPerPacket,
                 (double)kSamplesPerPacket * 1000.0 / (double)kFs);
  }

  std::vector<int16_t> out(kSamplesPerPacket);

  // meter raw
  uint16_t minv = 4095, maxv = 0;
  uint64_t last_meter_ns = now_ns();

  uint32_t pkt_id = 0;
  uint32_t meter_pkts = 0;
  uint32_t meter_bytes = 0;

  // debounce отпускания
  uint64_t release_start_ns = 0;

  auto ptt_is_pressed = []() -> bool {
    return (gpio_get_ptt_level() == 0); // 0=TX (нажато)
  };

  // если уже отпущено — не шлём
  if (!ptt_is_pressed()) {
    ::close(spi_fd);
    ::close(sockfd);
    gpio_set_activity_led(false);
    usleep(kRetryDelayUsec);
    return;
  }

  while (running.load()) {
    // Release debounce must not pause audio path:
    // while release is still "candidate", keep TX running to avoid net gaps.
    if (!ptt_is_pressed()) {
      const uint64_t now = now_ns();
      if (release_start_ns == 0) {
        release_start_ns = now;
      } else if (now - release_start_ns >= kPttReleaseDebounceNs) {
        std::fprintf(stderr, "[TX] BREAK: ptt released stable\n");
        break;
      }
    } else {
      release_start_ns = 0;
    }

    uint16_t pkt_min_u12 = 4095, pkt_max_u12 = 0;

    if (dma_active) {
      if (read_exact_fd(dma_fd, out.data(), kTxChunkBytes) != (ssize_t)kTxChunkBytes) {
        std::fprintf(stderr, "[TX] DMA read failed\n");
        break;
      }

      // Keep input shaping consistent with old path.
      for (size_t i = 0; i < kSamplesPerPacket; ++i) {
        int raw_est = clamp_int(((int)out[i] / 16) + 2048, 0, 4095);
        pkt_min_u12 = std::min<uint16_t>(pkt_min_u12, (uint16_t)raw_est);
        pkt_max_u12 = std::max<uint16_t>(pkt_max_u12, (uint16_t)raw_est);
        minv = std::min<uint16_t>(minv, (uint16_t)raw_est);
        maxv = std::max<uint16_t>(maxv, (uint16_t)raw_est);

        float x = (float)out[i] / 32768.0f;
        if (!hpf_primed) {
          hpf.x1 = x;
          hpf.y1 = 0.0f;
          hpf_primed = true;
        }
        x = hpf1_run(hpf, x) * kRecordGain;
        if (x > 0.999f) x = 0.999f;
        if (x < -0.999f) x = -0.999f;
        out[i] = (int16_t)lrintf(x * 32767.0f);
      }

      // Adaptive pacing for DMA path:
      // do not add extra delay if read() is already slower than chunk period.
      uint64_t now0 = now_ns();
      if (now0 + 50'000ull < t_next) {
        sleep_until_abs(t_next);
      }
      uint64_t now1 = now_ns();
      if (now1 > t_next) t_next = now1;
      t_next += chunk_period_ns;
    } else {
      // если мы заметно отстали — пересинхронизируемся
      uint64_t now0 = now_ns();
      if (now0 > t_next + period_ns * 4) t_next = now0;

      for (size_t i = 0; i < kSamplesPerPacket; ++i) {
        t_next += period_ns;
        sleep_until_abs(t_next);

        uint16_t raw = adc_read_u12(spi_fd, tr, rxb);
        minv = std::min(minv, raw);
        maxv = std::max(maxv, raw);
        pkt_min_u12 = std::min(pkt_min_u12, raw);
        pkt_max_u12 = std::max(pkt_max_u12, raw);

        float x = ((int)raw - 2048) / 2048.0f;
        if (!hpf_primed) {
          hpf.x1 = x;
          hpf.y1 = 0.0f;
          hpf_primed = true;
        }
        x = hpf1_run(hpf, x) * kRecordGain;
        if (x > 0.999f) x = 0.999f;
        if (x < -0.999f) x = -0.999f;
        out[i] = (int16_t)lrintf(x * 32767.0f);
      }
    }

    pkt_id++;
    meter_pkts++;
    meter_bytes += (uint32_t)kTxChunkBytes;

    if (tx_debug && (pkt_id == 1u || (pkt_id % 25u) == 0u)) {
      dump_s16_stats_tx(pkt_id, out.data(), out.size(), kFs, pkt_min_u12, pkt_max_u12);
    }

    uint64_t now = now_ns();
    if (tx_debug && (now - last_meter_ns > 1000000000ull)) {
      dump_tx_meter(dma_active ? "dma" : "spidev", pkt_id, meter_pkts, meter_bytes,
                    out.data(), out.size(), minv, maxv);
      minv = 4095; maxv = 0;
      meter_pkts = 0;
      meter_bytes = 0;
      last_meter_ns = now;
    }

    if (!send_all(sockfd, (const uint8_t*)out.data(), kTxChunkBytes)) {
      std::fprintf(stderr, "[TX] send_all failed\n");
      break;
    }
  }

  if (dma_fd >= 0) {
    (void)::ioctl(dma_fd, NAPI_AUDIO_DMA_IOC_STOP);
    ::close(dma_fd);
  }
  if (spi_fd >= 0) ::close(spi_fd);
  ::close(sockfd);
  gpio_set_activity_led(false);
  usleep(kRetryDelayUsec);
}
