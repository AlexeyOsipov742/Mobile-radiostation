// audioRxEth_PI.cpp (NaPi) — TCP -> MCP4822 (SPI) @ 12000 Hz, MONO S16LE
// FIX:
//  - recv_exact(): читаем РОВНО BUFFER_SIZE байт (TCP может дробить)
//  - Fs_in == Fs_out == 12000: НИКАКОГО ресэмплинга, просто джиттер-буфер + пейсинг DAC
//  - priming буфера ~40мс, чтобы убрать стартовый треск

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
#include <time.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sched.h>

#if __has_include(<gpiod.h>)
#include <gpiod.h>
#elif __has_include(<gpiod/gpiod.h>)
#include <gpiod/gpiod.h>
#endif

#ifndef SPI_NO_CS
#define SPI_NO_CS 0x40
#endif

#define CLAMP(v,lo,hi) ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))

// ---- time ----
static inline uint64_t now_ns(){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec;
}
static inline void ts_from_ns(uint64_t tns, struct timespec*ts){
  ts->tv_sec  = (time_t)(tns/1000000000ull);
  ts->tv_nsec = (long)(tns%1000000000ull);
}
static inline void busy_wait_to(uint64_t t_next){
  uint64_t now = now_ns();
  if (now + 5000ull < t_next){
    struct timespec ts; ts_from_ns(t_next - 3000ull, &ts);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
  }
  while ((now = now_ns()) < t_next) { /* spin */ }
}

// ---- GPIO CS ----
struct GpioCs {
  gpiod_chip *chip{nullptr};
  gpiod_line *line{nullptr};

  bool open(const char* chip_path, unsigned offset, const char* consumer){
    chip = gpiod_chip_open(chip_path);
    if (!chip){ fprintf(stderr,"gpiod_chip_open(%s) failed: %s\n", chip_path, strerror(errno)); return false; }
    line = gpiod_chip_get_line(chip, offset);
    if (!line){ fprintf(stderr,"gpiod_chip_get_line(%s,%u) failed: %s\n", chip_path, offset, strerror(errno)); gpiod_chip_close(chip); chip=nullptr; return false; }
    if (gpiod_line_request_output(line, consumer, 1) != 0){
      fprintf(stderr,"gpiod_line_request_output(%s,%u) failed: %s\n", chip_path, offset, strerror(errno));
      gpiod_chip_close(chip); chip=nullptr; line=nullptr; return false;
    }
    (void)gpiod_line_set_value(line, 1);
    return true;
  }
  inline void set(int v){ if (line) (void)gpiod_line_set_value(line, v); }
  void close(){
    if (line){ (void)gpiod_line_set_value(line, 1); gpiod_line_release(line); line=nullptr; }
    if (chip){ gpiod_chip_close(chip); chip=nullptr; }
  }
};

// ---- MCP4822 ----
static inline uint16_t mcp4822_word(bool chB, uint16_t u12){
  return (uint16_t)(((chB?1:0)<<15) | (1<<13) | (1<<12) | (u12 & 0x0FFF));
}
static inline uint16_t s16_to_u12(int16_t s, float vol){
  float x = (float)s / 32768.f;
  x *= vol;
  if (x > 0.999f) x = 0.999f;
  if (x < -0.999f) x = -0.999f;
  float y = x*0.5f + 0.5f;
  int v = (int)lrintf(y * 4095.f);
  if (v < 0) v = 0;
  if (v > 4095) v = 4095;
  return (uint16_t)v;
}

// ---- TCP recv_exact ----
static bool recv_exact(int fd, unsigned char* dst, int want){
  int got = 0;
  while (got < want){
    int n = ::recv(fd, dst + got, want - got, 0);
    if (n < 0){
      if (errno == EINTR) continue;
      perror("recv");
      return false;
    }
    if (n == 0) return false;
    got += n;
  }
  return true;
}

// ---- SPSC queue (mutex+cv, достаточно) ----
class Queue {
public:
  explicit Queue(size_t cap):buf(cap),cap(cap){}
  size_t push(const int16_t* data, size_t n){
    std::unique_lock<std::mutex> lk(m);
    size_t free = cap - size;
    size_t take = (n > free) ? free : n;
    for(size_t i=0;i<take;i++) buf[(head++)%cap] = data[i];
    size += take;
    lk.unlock();
    cv.notify_one();
    return take;
  }
  bool pop_one(int16_t &out){
    std::unique_lock<std::mutex> lk(m);
    cv.wait(lk, [&]{ return size>0 || stop; });
    if (size==0 && stop) return false;
    out = buf[(tail++)%cap];
    size--;
    return true;
  }
  size_t available(){
    std::lock_guard<std::mutex> lk(m);
    return size;
  }
  void clear(){
    std::lock_guard<std::mutex> lk(m);
    head = tail = size = 0;
  }
  void request_stop(){
    std::lock_guard<std::mutex> lk(m);
    stop = true;
    cv.notify_all();
  }
private:
  std::vector<int16_t> buf;
  size_t cap, head=0, tail=0, size=0;
  bool stop=false;
  std::mutex m;
  std::condition_variable cv;
};

class SpiDac12000 {
public:
  SpiDac12000(const char* dev, bool chB, uint32_t spi_hz, float vol, gpiod_line* cs_line)
    :dev(dev), chB(chB), spi_hz(spi_hz), vol(vol), cs(cs_line), q(1<<18) {}

  bool start(){
    sfd = ::open(dev, O_RDWR);
    if (sfd<0){ perror("open spidev"); return false; }

    uint8_t mode = SPI_MODE_0, bpw = 8;
    if (ioctl(sfd, SPI_IOC_WR_MODE, &mode)==-1){ perror("SPI_IOC_WR_MODE"); return false; }
    if (ioctl(sfd, SPI_IOC_WR_BITS_PER_WORD, &bpw)==-1){ perror("SPI_IOC_WR_BITS_PER_WORD"); return false; }
    if (ioctl(sfd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_hz)==-1){ perror("SPI_IOC_WR_MAX_SPEED_HZ"); return false; }

    running = true;
    th = std::thread(&SpiDac12000::render, this);
    return true;
  }

  void stop(){
    running = false;
    q.request_stop();
    if (th.joinable()) th.join();
    if (sfd>=0) ::close(sfd);
  }

  void begin_session(){
    q.clear();
    primed.store(false, std::memory_order_relaxed);
  }

  void push_mono(const int16_t* data, size_t samples){
    (void)q.push(data, samples);
  }

private:
  void render(){
    // RT приоритет (если разрешено в системе)
    struct sched_param sp{.sched_priority=50};
    (void)sched_setscheduler(0, SCHED_FIFO, &sp);

    const uint32_t Fs = (uint32_t)NAPI_LOCAL_FS; // 12000
    const uint64_t T = (uint64_t) llround(1e9 / (double)Fs);
    uint64_t t_next = now_ns();

    uint8_t tx[2];
    struct spi_ioc_transfer tr{};
    tr.tx_buf = (uint64_t)(uintptr_t)tx;
    tr.len = 2;
    tr.speed_hz = spi_hz;
    tr.bits_per_word = 8;

    primed.store(false, std::memory_order_relaxed);

    while (running){
      // priming ~40ms
      if (!primed.load(std::memory_order_relaxed)){
        const size_t need = (size_t)(Fs * 0.04);
        while (running && q.available() < need){
          t_next += T; busy_wait_to(t_next);
          uint16_t w = mcp4822_word(chB, 2048);
          tx[0]=(uint8_t)(w>>8); tx[1]=(uint8_t)w;
          if (cs) (void)gpiod_line_set_value(cs, 0);
          ioctl(sfd, SPI_IOC_MESSAGE(1), &tr);
          if (cs) (void)gpiod_line_set_value(cs, 1);
        }
        primed.store(true, std::memory_order_relaxed);
      }

      int16_t s16 = 0;
      if (!q.pop_one(s16)) break;

      t_next += T; busy_wait_to(t_next);

      uint16_t u12 = s16_to_u12(s16, vol);
      uint16_t w = mcp4822_word(chB, u12);
      tx[0]=(uint8_t)(w>>8); tx[1]=(uint8_t)w;

      if (cs) (void)gpiod_line_set_value(cs, 0);
      if (ioctl(sfd, SPI_IOC_MESSAGE(1), &tr) < 0) perror("SPI_IOC_MESSAGE");
      if (cs) (void)gpiod_line_set_value(cs, 1);
    }
  }

  const char* dev;
  bool chB;
  uint32_t spi_hz;
  float vol;
  int sfd{-1};
  gpiod_line* cs{nullptr};
  std::thread th;
  std::atomic<bool> running{false};
  std::atomic<bool> primed{false};
  Queue q;
};

// ===== network RX -> DAC =====
void audioRxEth_PI(unsigned char *buffer, std::atomic<bool> &running)
{
  // мы ждём по сети: MONO S16LE @ 12000
  const int channels = 1;

  // DAC/SPI
  const char* SPI_DEV = NAPI_SPI_DEV;
  const bool  DAC_CHB = false;
  uint32_t    SPI_HZ  = 1000000;
  float       VOL     = 0.5f;

  GpioCs dac_cs;
  if (!dac_cs.open(NAPI_DAC_CS_CHIP, (unsigned)NAPI_DAC_CS_LINE, "napi-dac-cs")){
    fprintf(stderr,"[RX] DAC-CS init failed\n");
    return;
  }

  SpiDac12000 dac(SPI_DEV, DAC_CHB, SPI_HZ, VOL, dac_cs.line);
  if (!dac.start()){
    fprintf(stderr,"[RX] DAC start failed\n");
    dac_cs.close();
    return;
  }

  int sockfd=-1, newsockfd=-1;
  struct sockaddr_in serv{}, cli{};
  socklen_t clilen=0;

  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0){
    perror("socket");
    dac.stop(); dac_cs.close();
    return;
  }
  int optval=1;
  (void)setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

  memset(&serv,0,sizeof(serv));
  serv.sin_family = AF_INET;
  serv.sin_addr.s_addr = INADDR_ANY;
  serv.sin_port = htons(PORT);

  if (bind(sockfd,(struct sockaddr*)&serv, sizeof(serv))<0){
    perror("bind");
    close(sockfd);
    dac.stop(); dac_cs.close();
    return;
  }
  listen(sockfd,5);
  clilen=sizeof(cli);

  fd_set rfds;
  struct timeval tv{};

  gpio_set_activity_led(true);

  while (running){
    FD_ZERO(&rfds);
    FD_SET(sockfd,&rfds);
    tv.tv_sec=0; tv.tv_usec=100000;

    int sel = select(sockfd+1,&rfds,NULL,NULL,&tv);
    if (sel<0){ perror("select"); break; }
    if (sel==0) continue;

    newsockfd = accept(sockfd,(struct sockaddr*)&cli,&clilen);
    if (newsockfd<0){ perror("accept"); continue; }

    fprintf(stderr,"[RX] client connected\n");
    dac.begin_session();

    while (running){
      if (!recv_exact(newsockfd, buffer, BUFFER_SIZE)){
        fprintf(stderr,"[RX] client disconnected\n");
        close(newsockfd); newsockfd=-1;
        break;
      }

      // фильтр командных пакетов (если вдруг прилетит)
      if (BUFFER_SIZE>=2 && buffer[0]=='K' && buffer[1]=='N'){
        continue;
      }

      const int samples = BUFFER_SIZE / 2;
      dac.push_mono((const int16_t*)buffer, (size_t)samples);
    }
  }

  if (newsockfd>=0) close(newsockfd);
  if (sockfd>=0) close(sockfd);

  gpio_set_activity_led(false);
  dac.stop();
  dac_cs.close();
  memset(buffer,0,BUFFER_SIZE);
}

