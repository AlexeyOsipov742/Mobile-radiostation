// audioRxEth_PI.cpp (NaPi) — TCP -> MCP4822 (SPI DAC)
// Цель фикса:
//  - Работаем "родной" частотой NAPI_LOCAL_FS (обычно 12000) и на входе ожидаем то же (но умеем ресэмплить, если надо)
//  - RX ДОЛЖЕН уметь ВЫЙТИ, когда нажали PTT (gpio_get_ptt_level()==0), чтобы audio.cpp переключился на TX
//  - Блокирующий recv не должен "вешать" переключение -> SO_RCVTIMEO + recv_exact() понимает timeout
//  - Джиттер-буфер + мягкий mute при underrun
//  - Без goto (чтобы не ловить C++ "crosses initialization")
//
// FIX (пункт 2):
//  - Исправлен "залипающий stop" в очереди Ring между сессиями/остановками
//  - Добавлен watchdog: если по TCP долго нет данных (даже при таймаутах SO_RCVTIMEO), считаем сессию умершей и выходим,
//    чтобы не копить деградацию/зависшие состояния
//
// Остальное не трогал.

#include "TxRx.h"

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <algorithm>
#include <cmath>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

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

#ifndef SPI_NO_CS
  #define SPI_NO_CS 0x40
#endif

static inline int clampi(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// ---------- timing ----------
static inline uint64_t now_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static inline void ts_from_ns(uint64_t tns, timespec *ts) {
  ts->tv_sec  = (time_t)(tns / 1000000000ull);
  ts->tv_nsec = (long)(tns % 1000000000ull);
}

static inline void busy_wait_to(uint64_t t_next) {
  uint64_t now = now_ns();
  // если далеко — чуть поспим, потом докрутим спином
  if (now + 5000ull < t_next) {
    timespec ts{};
    ts_from_ns(t_next - 3000ull, &ts);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
  }
  while ((now = now_ns()) < t_next) { /* spin */ }
}

// ---------- GPIO CS ----------
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
      gpiod_chip_close(chip);
      chip = nullptr;
      return false;
    }
    if (gpiod_line_request_output(line, consumer, 1) != 0) {
      std::fprintf(stderr, "gpiod_line_request_output(%s,%u) failed: %s\n", chip_path, offset, std::strerror(errno));
      gpiod_chip_close(chip);
      chip = nullptr;
      line = nullptr;
      return false;
    }
    (void)gpiod_line_set_value(line, 1);
    return true;
  }

  inline void set(int v) {
    if (line) (void)gpiod_line_set_value(line, v);
  }

  void close() {
    if (line) {
      (void)gpiod_line_set_value(line, 1);
      gpiod_line_release(line);
      line = nullptr;
    }
    if (chip) {
      gpiod_chip_close(chip);
      chip = nullptr;
    }
  }
};

// ---------- MCP4822 ----------
static inline uint16_t mcp4822_word(bool chB, uint16_t u12) {
  // [15]=A/B, [14]=BUF(0), [13]=GA(1=1x), [12]=SHDN(1=active)
  return (uint16_t)(((chB ? 1 : 0) << 15) | (1 << 13) | (1 << 12) | (u12 & 0x0FFF));
}

static inline uint16_t s16_to_u12(int16_t s, float vol) {
  float x = (float)s / 32768.0f;
  x *= vol;
  if (x > 0.999f) x = 0.999f;
  if (x < -0.999f) x = -0.999f;
  float y = x * 0.5f + 0.5f; // [-1..1] -> [0..1]
  int v = (int)lrintf(y * 4095.0f);
  v = clampi(v, 0, 4095);
  return (uint16_t)v;
}

// ---------- SPSC-ish ring (mutex+cv, достаточно для нашего случая) ----------
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

  // FIX: важно сбрасывать stop при новом старте/сессии,
  // иначе pop_one() может навсегда начать возвращать false.
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
  SpiDac(const char *spi_dev,
         bool chB,
         uint32_t spi_hz,
         float vol,
         uint32_t fs_in,
         uint32_t fs_out,
         gpiod_line *cs_line)
      : dev(spi_dev),
        chB(chB),
        spi_hz(spi_hz),
        vol(vol),
        Fs_in(fs_in),
        Fs_out(fs_out),
        cs(cs_line),
        q((size_t)fs_out * 1) // 1 сек буфера
  {}

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
      ::close(sfd); sfd = -1;
      return false;
    }
    if (ioctl(sfd, SPI_IOC_WR_BITS_PER_WORD, &bpw) == -1) {
      std::perror("SPI_IOC_WR_BITS_PER_WORD");
      ::close(sfd); sfd = -1;
      return false;
    }
    if (ioctl(sfd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_hz) == -1) {
      std::perror("SPI_IOC_WR_MAX_SPEED_HZ");
      ::close(sfd); sfd = -1;
      return false;
    }

    // FIX: сбросить stop перед запуском потока
    q.reset_stop();

    running.store(true);
    th = std::thread(&SpiDac::render, this);
    return true;
  }

  void stop() {
    running.store(false);
    q.request_stop();
    if (th.joinable()) th.join();
    if (sfd >= 0) ::close(sfd);
    sfd = -1;
  }

  void begin_session(uint32_t newFsIn) {
    // FIX: новая сессия — обязательно снять stop, иначе pop_one() может "умереть" после прошлой остановки
    q.reset_stop();

    fs_in_req.store(newFsIn, std::memory_order_relaxed);
    need_reset.store(true, std::memory_order_release);
  }

  void push_frames(const int16_t *data, size_t frames, int channels) {
    if (!frames) return;

    if (channels == 1) {
      (void)q.push(data, frames);
      return;
    }

    tmp.resize(frames);
    for (size_t i = 0; i < frames; i++) tmp[i] = data[i * (size_t)channels]; // берём левый
    (void)q.push(tmp.data(), frames);
  }

private:
  void render() {
    // realtime-приоритет (не обязателен, но помогает ровности SPI)
    {
      sched_param sp{};
      sp.sched_priority = 50;
      if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        // не фатально
      }
      (void)mlockall(MCL_CURRENT | MCL_FUTURE);
    }

    const uint64_t T = (uint64_t)llround(1e9 / (double)Fs_out);
    uint64_t t_next  = now_ns();

    uint8_t tx[2]{0, 0};
    spi_ioc_transfer tr{};
    tr.tx_buf = (uintptr_t)tx;
    tr.len = 2;
    tr.speed_hz = spi_hz;
    tr.bits_per_word = 8;
    tr.cs_change = 0;
    tr.delay_usecs = 0;

    // ресэмпл (если Fs_in != Fs_out): простой линейный
    double ratio = (double)Fs_in / (double)Fs_out;
    double pos = 0.0;

    float y0 = 0.0f, y1 = 0.0f;
    bool primed = false;

    float env = 0.0f;
    const float fadeUp   = 1.0f / (0.010f * (float)Fs_out);
    const float fadeDown = 1.0f / (0.010f * (float)Fs_out);

    auto write_u12 = [&](uint16_t u12) {
      uint16_t w = mcp4822_word(chB, u12);
      tx[0] = (uint8_t)(w >> 8);
      tx[1] = (uint8_t)(w & 0xFF);
      if (cs) (void)gpiod_line_set_value(cs, 0);
      (void)ioctl(sfd, SPI_IOC_MESSAGE(1), &tr);
      if (cs) (void)gpiod_line_set_value(cs, 1);
    };

    auto reset_state = [&](){
      uint32_t req = fs_in_req.load(std::memory_order_relaxed);
      if (req) Fs_in = req;

      ratio = (double)Fs_in / (double)Fs_out;
      pos = 0.0;
      y0 = y1 = 0.0f;
      env = 0.0f;
      primed = false;
      q.clear();
      t_next = now_ns();
    };

    while (running.load()) {
      if (need_reset.exchange(false, std::memory_order_acquire)) {
        reset_state();
      }

      // прайминг: накопим немного входа, чтобы не сразу шуметь от голода
      if (!primed) {
        const size_t need = (size_t)((double)Fs_in * 0.04); // 40мс
        while (running.load() && q.available() < need) {
          t_next += T;
          busy_wait_to(t_next);
          write_u12(2048); // mid
          if (need_reset.load(std::memory_order_relaxed)) break;
        }
        if (!running.load() || need_reset.load(std::memory_order_relaxed)) continue;

        int16_t s{};
        if (!q.pop_one(s)) continue;
        y0 = (float)s / 32768.0f;
        if (!q.pop_one(s)) continue;
        y1 = (float)s / 32768.0f;
        pos = 0.0;
        primed = true;
        continue;
      }

      // underrun -> mid + fade down
      if (q.available() < 2) {
        env = std::max(0.0f, env - fadeDown);
        t_next += T;
        busy_wait_to(t_next);
        write_u12(2048);
        continue;
      }

      pos += ratio;

      while (pos >= 1.0) {
        y0 = y1;
        int16_t s{};
        if (!q.pop_one(s)) { pos = std::fmod(pos, 1.0); break; }
        y1 = (float)s / 32768.0f;
        pos -= 1.0;
      }

      float mu = (float)pos;
      float y  = y0 + (y1 - y0) * mu;

      // env по запасу
      if (q.available() < (size_t)((double)Fs_in * 0.005))
        env = std::max(0.0f, env - fadeDown);
      else
        env = std::min(1.0f, env + fadeUp);

      float yenv = y * env;

      int16_t s16 = (int16_t)clampi((int)lrintf(yenv * 32767.0f), -32768, 32767);

      t_next += T;
      busy_wait_to(t_next);
      write_u12(s16_to_u12(s16, vol));
    }

    // на выходе заглушим
    write_u12(2048);
  }

  const char *dev;
  bool chB;
  uint32_t spi_hz;
  float vol;

  uint32_t Fs_in;
  uint32_t Fs_out;

  int sfd{-1};
  gpiod_line *cs{nullptr};

  std::thread th;
  std::atomic<bool> running{false};

  Ring q;
  std::vector<int16_t> tmp;

  std::atomic<bool> need_reset{false};
  std::atomic<uint32_t> fs_in_req{0};
};

// ---------- recv_exact with timeout-awareness ----------
static bool recv_exact(int fd, unsigned char *dst, int want) {
  int got = 0;
  while (got < want) {
    int n = ::recv(fd, dst + got, want - got, 0);
    if (n < 0) {
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) return false; // timeout
      std::perror("recv");
      return false;
    }
    if (n == 0) return false; // disconnect
    got += n;
  }
  return true;
}

// =======================================================
//                    NaPi RX entry
// =======================================================
void audioRxEth_client(unsigned char *buffer, std::atomic<bool> &running) {
  // Если уже нажали PTT — не входить в RX
  if (!running.load()) return;
  if (gpio_get_ptt_level() == 0) return;

  // Настройки формата сети
  const uint32_t Fs_out = (uint32_t)NAPI_LOCAL_FS;      // например 12000
  const uint32_t Fs_in  = (uint32_t)NAPI_LOCAL_FS;      // ожидаем 12000 от Raspberry
  const int channels    = (int)NAPI_RX_NET_CHANNELS;    // 1 или 2 (берём левый)

  // SPI DAC
  const char *SPI_DEV = NAPI_SPI_DEV;
  const bool  DAC_CHB = false;     // MCP4822 channel A
  uint32_t    SPI_HZ  = 1000000;   // безопасно
  float       VOL     = 0.5f;

  // GPIO-CS для DAC
  GpioCs dac_cs;
  if (!dac_cs.open(NAPI_DAC_CS_CHIP, (unsigned)NAPI_DAC_CS_LINE, "napi-dac-cs")) {
    std::fprintf(stderr, "[RX] DAC-CS init failed\n");
    return;
  }

  SpiDac dac(SPI_DEV, DAC_CHB, SPI_HZ, VOL, Fs_in, Fs_out, dac_cs.line);
  if (!dac.start()) {
    std::fprintf(stderr, "[RX] DAC start failed\n");
    dac_cs.close();
    return;
  }

  int sockfd = -1;
  int newsockfd = -1;

  sockfd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    std::perror("socket");
    dac.stop();
    dac_cs.close();
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
    dac.stop();
    dac_cs.close();
    return;
  }

  if (listen(sockfd, 5) < 0) {
    std::perror("listen");
    ::close(sockfd);
    dac.stop();
    dac_cs.close();
    return;
  }

  gpio_set_activity_led(true);

  // accept loop: должен уметь выйти по PTT
  while (running.load()) {
    if (gpio_get_ptt_level() == 0) break;

    // accept с небольшим таймаутом через select (чтобы быстро реагировать на PTT)
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sockfd, &rfds);

    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms

    int sel = select(sockfd + 1, &rfds, nullptr, nullptr, &tv);
    if (sel < 0) {
      if (errno == EINTR) continue;
      std::perror("select");
      break;
    }
    if (sel == 0) continue;

    sockaddr_in cli{};
    socklen_t clilen = sizeof(cli);
    newsockfd = accept(sockfd, (sockaddr *)&cli, &clilen);
    if (newsockfd < 0) {
      if (errno == EINTR) continue;
      std::perror("accept");
      continue;
    }

    // Ставим таймаут на recv, чтобы recv_exact не блокировал переключение PTT
    {
      timeval rcvto{};
      rcvto.tv_sec = 0;
      rcvto.tv_usec = 20000; // 20ms
      (void)setsockopt(newsockfd, SOL_SOCKET, SO_RCVTIMEO, &rcvto, sizeof(rcvto));
    }

    // новая сессия для DAC (сброс буфера)
    dac.begin_session(Fs_in);

    // FIX: watchdog "нет данных" — если долго не получаем ни одного полного блока, закрываем сессию
    const uint64_t kNoDataTimeoutNs = 2ull * 1000000000ull; // 2 секунды без данных
    uint64_t last_ok_ns = now_ns();

    // RX loop
    while (running.load()) {
      if (gpio_get_ptt_level() == 0) { // нажали PTT -> выходим из RX
        break;
      }

      if (!recv_exact(newsockfd, buffer, BUFFER_SIZE)) {
        // timeout -> продолжаем, но следим за общим временем "нет данных"
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          if (now_ns() - last_ok_ns > kNoDataTimeoutNs) {
            // считаем, что сессия умерла/подвисла
            break;
          }
          continue;
        }
        // иначе — disconnect/ошибка
        break;
      }

      last_ok_ns = now_ns();

      // отфильтровать управляющее "KN" (если оно реально прилетает в этот канал)
      if (buffer[0] == 'K' && buffer[1] == 'N') {
        continue;
      }

      // frames из байт
      const int bytes = BUFFER_SIZE;
      const int frame_bytes = 2 * channels; // S16LE
      int frames = bytes / frame_bytes;
      if (frames <= 0) continue;

      dac.push_frames((const int16_t *)buffer, (size_t)frames, channels);
    }

    // закрываем клиента
    if (newsockfd >= 0) {
      ::close(newsockfd);
      newsockfd = -1;
    }

    if (gpio_get_ptt_level() == 0) break;
  }

  if (newsockfd >= 0) ::close(newsockfd);
  if (sockfd >= 0) ::close(sockfd);

  gpio_set_activity_led(false);
  dac.stop();
  dac_cs.close();
  std::memset(buffer, 0, BUFFER_SIZE);
}
