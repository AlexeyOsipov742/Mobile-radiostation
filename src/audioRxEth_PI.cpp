// audioRxEth_PI.cpp — прием аудио по TCP и воспроизведение через MCP4822 (SPI)
// Вход: S16_LE, моно или стерео, Fs_in = SAMPLERATE (44100/48000 и т.п.)
// Выход в ЦАП: фиксированные 11025 Гц. Адаптивный ресэмплер + LPF + TPDF-dither.
// Джиттер-буфер, мягкий fade-in/out, анти-underrun.
// Сборка: убрать -lasound; добавить -pthread.

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
#include <fcntl.h>
#include <unistd.h>
#include <sched.h>
#include <time.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#define CLAMP(v,lo,hi) ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))

// ---- таймеры ----
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

// ---- MCP4822 ----
static inline uint16_t mcp4822_word(bool chB, uint16_t u12){
  return (uint16_t)(((chB?1:0)<<15) | (1<<13) | (1<<12) | (u12 & 0x0FFF));
}
static inline uint16_t s16_to_u12_dither(int16_t s, float vol, uint32_t *rng){
  float x = (float)s / 32768.f; x *= vol;
  if (x > 0.999f) { x = 0.999f; }
  else if (x < -0.999f) { x = -0.999f; }

  *rng = *rng * 1664525u + 1013904223u; float u1 = ((*rng>>8)&0xFFFF)/65535.0f;
  *rng = *rng * 1664525u + 1013904223u; float u2 = ((*rng>>8)&0xFFFF)/65535.0f;
  float d = (u1 + u2 - 1.0f) / 4095.0f;
  float y = (x*0.5f + 0.5f) + d;
  int v = (int)lrintf(y * 4095.f);
  if (v < 0) { v = 0; }
  else if (v > 4095) { v = 4095; }
  return (uint16_t)v;
}

// ---- простая biquad LPF для защиты перед downsample (Fc≈4.8 кГц при 44.1к) ----
struct BQ { float b0,b1,b2,a1,a2; float z1,z2; };
static inline void bq_init(BQ &q, float b0,float b1,float b2,float a1,float a2){
  q.b0=b0; q.b1=b1; q.b2=b2; q.a1=a1; q.a2=a2; q.z1=q.z2=0.0f;
}
static inline float bq_run(BQ &q, float x){
  float y = q.b0*x + q.z1;
  q.z1 = q.b1*x + q.z2 - q.a1*y;
  q.z2 = q.b2*x - q.a2*y;
  return y;
}
// Коэфы под Fs≈44100; при 48000 работает как мягкий LPF (для речи достаточно)
static inline BQ bq_lowpass_4k8(){
  BQ q; bq_init(q, 0.06745527f, 0.13491054f, 0.06745527f, -1.1429805f, 0.41280160f); return q;
}

// ---- SPSC кольцо ----
class Spsc {
public:
  explicit Spsc(size_t cap):buf(cap),cap(cap){}
  size_t push(const int16_t* data, size_t n){
    std::unique_lock<std::mutex> lk(m);
    size_t free = cap - size;
    size_t take = n>free?free:n;
    for(size_t i=0;i<take;i++) buf[(head++)%cap] = data[i];
    size += take; lk.unlock(); cv.notify_one(); return take;
  }
  // блокирующее извлечение 1 семпла
  bool pop_one(int16_t &out){
    std::unique_lock<std::mutex> lk(m);
    cv.wait(lk, [&]{ return size>0 || stop; });
    if (size==0 && stop) return false;
    out = buf[(tail++)%cap]; size--; return true;
  }
  // неблокирующий "peek": сколько есть
  size_t available(){
    std::lock_guard<std::mutex> lk(m); return size;
  }
  void clear(){
    std::lock_guard<std::mutex> lk(m);
    head = tail = size = 0;
  }
  void request_stop(){ std::lock_guard<std::mutex> lk(m); stop=true; cv.notify_all(); }
private:
  std::vector<int16_t> buf; size_t cap, head=0, tail=0, size=0; bool stop=false;
  std::mutex m; std::condition_variable cv;
};

// ---- плеер SPI с адаптивным ресэмплером Fs_in -> 11025 ----
class SpiDac {
public:
  SpiDac(const char* dev, bool chB, uint32_t spi_hz, float vol,
         uint32_t Fs_in, uint32_t Fs_out=11025)
    :dev(dev), chB(chB), spi_hz(spi_hz), vol(vol),
     Fs_in(Fs_in), Fs_out(Fs_out), inq(1<<18) {}

  bool start(){
    sfd = open(dev, O_RDWR);
    if (sfd<0){ perror("open spidev"); return false; }
    uint8_t mode=SPI_MODE_0, bpw=8;
    if (ioctl(sfd, SPI_IOC_WR_MODE, &mode)==-1){ perror("SPI_IOC_WR_MODE"); return false; }
    if (ioctl(sfd, SPI_IOC_WR_BITS_PER_WORD, &bpw)==-1){ perror("SPI_IOC_WR_BITS_PER_WORD"); return false; }
    if (ioctl(sfd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_hz)==-1){ perror("SPI_IOC_WR_MAX_SPEED_HZ"); return false; }
    running=true; th = std::thread(&SpiDac::render, this); return true;
  }
  void stop(){
    running=false; inq.request_stop();
    if(th.joinable()) th.join();
    if(sfd>=0) close(sfd);
  }

  // сигнал о начале новой сессии (сброс буферов/фаз/фильтра)
  void begin_session(uint32_t newFsIn){
    fs_in_req.store(newFsIn, std::memory_order_relaxed);
    need_reset.store(true, std::memory_order_release);
  }

  // Подаём вход моно/стерео @Fs_in. Берём левый.
  void push(const int16_t* data, size_t frames, int channels){
    if (!frames) return;
    if (channels==1){ (void)inq.push(data, frames); }
    else{
      tmp.resize(frames);
      for(size_t i=0;i<frames;i++) tmp[i]=data[i*2];
      (void)inq.push(tmp.data(), frames);
    }
  }
private:
  void render(){
    struct sched_param sp={.sched_priority=50}; sched_setscheduler(0, SCHED_FIFO, &sp);

    const uint64_t T = (uint64_t) llround(1e9 / (double)Fs_out);
    uint64_t t_next = now_ns();

    uint8_t tx[2]; struct spi_ioc_transfer tr{};
    tr.tx_buf = (uint64_t)(uintptr_t)tx; tr.len=2;
    tr.speed_hz=spi_hz; tr.bits_per_word=8; tr.cs_change=0; tr.delay_usecs=0;

    BQ lp = bq_lowpass_4k8();
    double ratio = (double)Fs_in / (double)Fs_out;
    double pos = 0.0;

    float y0=0.0f, y1=0.0f;
    float gainEnv = 0.0f;
    const float fadeUpPerSample   = 1.0f / (0.010f * Fs_out);
    const float fadeDownPerSample = 1.0f / (0.010f * Fs_out);
    uint32_t rng=0x12345678u;
    size_t underruns=0;

    auto do_reset = [&](){
      uint32_t req = fs_in_req.load(std::memory_order_relaxed);
      if (req) Fs_in = req;
      ratio = (double)Fs_in / (double)Fs_out;

      inq.clear();
      lp = bq_lowpass_4k8();
      pos = 0.0;
      y0 = y1 = 0.0f;
      gainEnv = 0.0f;
      primed.store(false, std::memory_order_relaxed);
      t_next = now_ns();
    };

    primed.store(false, std::memory_order_relaxed);

    while (running){
      if (need_reset.exchange(false, std::memory_order_acquire)){
        do_reset();
      }

      // прайминг буфера и начальных опорных точек
      if (!primed.load(std::memory_order_relaxed)){
        const size_t priming = (size_t)(Fs_in * 0.04); // ~40мс
        while (running && inq.available() < priming){
          t_next += T; busy_wait_to(t_next);
          uint16_t w = mcp4822_word(chB, 2048);
          tx[0]=(uint8_t)(w>>8); tx[1]=(uint8_t)w;
          ioctl(sfd, SPI_IOC_MESSAGE(1), &tr);
          if (need_reset.load()) break;
        }
        if (running && !need_reset.load()){
          int16_t s;
          if (!inq.pop_one(s)) continue;
          y0 = bq_run(lp, (float)s/32768.0f);
          if (!inq.pop_one(s)) continue;
          y1 = bq_run(lp, (float)s/32768.0f);
          pos = 0.0;
          primed.store(true, std::memory_order_relaxed);
        }
        continue;
      }

      // если голодаем — мягкая середина
      if (inq.available() < 2){
        underruns++;
        if (gainEnv > 0.0f) gainEnv = std::max(0.0f, gainEnv - fadeDownPerSample);
        t_next += T; busy_wait_to(t_next);
        uint16_t w = mcp4822_word(chB, 2048);
        tx[0]=(uint8_t)(w>>8); tx[1]=(uint8_t)w;
        ioctl(sfd, SPI_IOC_MESSAGE(1), &tr);
        continue;
      }

      // ВАЖНО: сначала продвинуть фазу
      pos += ratio;

      // перенос окна [y0..y1], пока фаза >= 1
      while (pos >= 1.0){
        y0 = y1;
        int16_t s;
        if (!inq.pop_one(s)) { pos = std::fmod(pos, 1.0); break; }
        y1 = bq_run(lp, (float)s/32768.0f);
        pos -= 1.0;
      }

      // интерполяция
      float mu = (float)pos;
      float y  = y0 + (y1 - y0) * mu;

      // fade по запасу
      if (inq.available() < (size_t)(Fs_in*0.005))
        gainEnv = std::max(0.0f, gainEnv - fadeDownPerSample);
      else
        gainEnv = std::min(1.0f, gainEnv + fadeUpPerSample);

      float y_env = y * gainEnv;

      // тайминг и вывод
      t_next += T; busy_wait_to(t_next);
      int16_t s16 = (int16_t)CLAMP((int)lrintf(y_env*32767.0f), -32768, 32767);
      uint16_t u12 = s16_to_u12_dither(s16, vol, &rng);
      uint16_t w = mcp4822_word(chB, u12);
      tx[0]=(uint8_t)(w>>8); tx[1]=(uint8_t)w;
      if (ioctl(sfd, SPI_IOC_MESSAGE(1), &tr) < 0) perror("SPI_IOC_MESSAGE");
    }

    if (underruns) fprintf(stderr,"[DAC] underruns=%zu\n", underruns);
  }

  const char* dev; bool chB; uint32_t spi_hz; float vol;
  uint32_t Fs_in, Fs_out; int sfd{-1};
  std::thread th; std::atomic<bool> running{false};
  Spsc inq; std::vector<int16_t> tmp;

  // reset-сигналы/флаги
  std::atomic<bool> need_reset{false};
  std::atomic<uint32_t> fs_in_req{0};
  std::atomic<bool> primed{false};
};

// ======== СЕТЕВОЙ RX + ПОДАЧА В ПЛЕЕР ========
void audioRxEth_PI(unsigned char *buffer, std::atomic<bool> &running)
{
  // сеть
  int sockfd=-1, newsockfd=-1;
  struct sockaddr_in serv{}, cli{}; socklen_t clilen=0;

  // формат входа (из проекта)
#ifdef SAMPLERATE
  const uint32_t Fs_in = (uint32_t)SAMPLERATE;
#else
  const uint32_t Fs_in = 44100u;   // дефолт
#endif
  const int channels = 1;          // если стерео — поставь 2

  // SPI ЦАП
  const char* SPI_DEV = "/dev/spidev2.0"; // поменяй при необходимости
  const bool  DAC_CHB = false;            // канал MCP4822: A=false, B=true
  uint32_t    SPI_HZ  = 2000000;          // можно 2–8 МГц
  float       VOL     = 0.5f;

  SpiDac dac(SPI_DEV, DAC_CHB, SPI_HZ, VOL, Fs_in);
  if (!dac.start()){ fprintf(stderr,"DAC start failed\n"); return; }

  // сервер (RAW PCM без заголовков)
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0){ perror("socket"); dac.stop(); return; }
  int optval=1; setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
  memset(&serv,0,sizeof(serv));
  serv.sin_family = AF_INET; serv.sin_addr.s_addr = INADDR_ANY; serv.sin_port = htons(PORT);
  if (bind(sockfd,(struct sockaddr*)&serv, sizeof(serv))<0){ perror("bind"); close(sockfd); dac.stop(); return; }
  listen(sockfd,5); clilen=sizeof(cli);

  fd_set rfds; struct timeval tv{};

  while (running){
    FD_ZERO(&rfds); FD_SET(sockfd,&rfds);
    tv.tv_sec=0; tv.tv_usec=100000;
    int sel = select(sockfd+1,&rfds,NULL,NULL,&tv);
    if (sel<0){ perror("select"); break; }
    if (sel==0) continue;

    newsockfd = accept(sockfd,(struct sockaddr*)&cli,&clilen);
    if (newsockfd<0){ perror("accept"); continue; }
    fprintf(stderr,"[RX] client connected\n");

    // сигнал плееру начать новую сессию (очистка/сброс)
    dac.begin_session(Fs_in);

    // читаем кусками и сразу отправляем в плеер
    while (running){
      int n = recv(newsockfd, buffer, BUFFER_SIZE, 0);
      if (n <= 0){
        if (n<0) perror("recv");
        close(newsockfd); newsockfd=-1;
        fprintf(stderr,"[RX] client disconnected\n");
        break;
      }

      // если бывают командные пакеты типа 'K''N' — отфильтровать здесь
      if (n>=2 && buffer[0]=='K' && buffer[1]=='N') {
        continue;
      }

      int frames = n / (2*channels); // S16_LE
      dac.push((const int16_t*)buffer, frames, channels);
    }
  }

  if (newsockfd>=0) close(newsockfd);
  if (sockfd>=0)    close(sockfd);
  dac.stop();
  memset(buffer,0,BUFFER_SIZE);
}
