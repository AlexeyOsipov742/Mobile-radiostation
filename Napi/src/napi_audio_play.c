// wav_11025.c — WAV (S16_LE, stereo→L) -> MCP4822 (A|B) через spidev
// Жёсткий фиксированный вывод @ 11025 Гц (44100/4) без калибровок.
// Перед прореживанием ×4: biquad low-pass ~4.8 кГц, затем TPDF-dither к 12 бит.
// Тайминг: TIMER_ABSTIME + короткий spin до цели; 1 ioctl на семпл.
//
// Сборка: gcc -O2 -Wall -o wav_11025 wav_11025.c -lm
// Запуск:  ./wav_11025 <wav> <spidev> <A|B> <spi_hz> [volume]

#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <inttypes.h>
#include <sched.h>

#define CLAMP(v,lo,hi) ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))

typedef struct {
  uint32_t chunk_id, chunk_size, format;        // "RIFF", size, "WAVE"
  uint32_t sub1_id, sub1_size;                  // "fmt "
  uint16_t audio_format, num_channels;
  uint32_t sample_rate, byte_rate;
  uint16_t block_align, bits_per_sample;
} __attribute__((packed)) WavHdr;

static inline uint64_t now_ns(void){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec;
}

static void ts_from_ns(uint64_t tns, struct timespec*ts){
  ts->tv_sec  = (time_t)(tns/1000000000ull);
  ts->tv_nsec = (long)(tns%1000000000ull);
}

static int find_data(int fd, uint32_t *sz){
  while (1){
    uint32_t id=0, s=0;
    if (read(fd,&id,4)!=4) return -1;
    if (read(fd,&s,4)!=4)  return -1;
    if (id==0x61746164){ *sz=s; return 0; } // "data"
    if (lseek(fd, s + (s & 1), SEEK_CUR) < 0) return -1;
  }
}

static int open_spi(const char*dev, uint32_t *hz_inout){
  int fd = open(dev, O_RDWR);
  if (fd < 0){ perror("open spidev"); return -1; }
  uint8_t mode = SPI_MODE_0, bpw = 8;
  if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1){ perror("SPI_IOC_WR_MODE"); goto fail; }
  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) == -1){ perror("SPI_IOC_WR_BITS_PER_WORD"); goto fail; }
  uint32_t hz = *hz_inout;
  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) == -1){ perror("SPI_IOC_WR_MAX_SPEED_HZ"); goto fail; }
  uint32_t rd=0; if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &rd) == -1) rd=hz;
  if (rd && rd!=*hz_inout) fprintf(stderr,"WARN: requested %u Hz, driver %u Hz\n", *hz_inout, rd);
  if (rd) *hz_inout = rd;
  return fd;
fail:
  close(fd); return -1;
}

// S16 -> 12-бит беззнак (MCP4822), volume, TPDF dither ±0.5 LSB
static inline uint16_t s16_to_u12_dither(int16_t s, float vol, uint32_t *rng){
  // scale & clamp
  float x = (float)s / 32768.f; x *= vol;
  if (x > 0.999f) x = 0.999f; if (x < -0.999f) x = -0.999f;
  // TPDF dither: (u1+u2-1) * 1 LSB
  // fast LCG
  *rng = *rng * 1664525u + 1013904223u;
  float u1 = ((*rng >> 8) & 0xFFFF) / 65535.0f;
  *rng = *rng * 1664525u + 1013904223u;
  float u2 = ((*rng >> 8) & 0xFFFF) / 65535.0f;
  float d  = (u1 + u2 - 1.0f) / 4095.0f; // 1 LSB @ 12-bit
  float y = (x*0.5f + 0.5f) + d;
  int v = (int)lrintf(y * 4095.f);
  if (v < 0) v = 0; if (v > 4095) v = 4095;
  return (uint16_t)v;
}

// MCP4822 word: [15]=B, [14]=BUF=0, [13]=GA=1 (1x), [12]=SHDN=1, [11:0]=DATA
static inline uint16_t mcp4822_word(int chB, uint16_t u12){
  return (uint16_t)(((chB?1:0)<<15) | (1<<13) | (1<<12) | (u12 & 0x0FFF));
}

/* --- Простая би-квад НЧ для децимации ×4 (Fs_in=44100, Fc≈4800 Гц) ---
   Коэффициенты рассчитаны offline для Fs=44100, Fc≈4800, Q≈0.707.
   Нормализовано под Direct Form I/II (a0=1). */
typedef struct { float b0,b1,b2,a1,a2; float z1,z2; } BQ;

static void bq_init(BQ *q, float b0,float b1,float b2,float a1,float a2){
  q->b0=b0; q->b1=b1; q->b2=b2; q->a1=a1; q->a2=a2; q->z1=q->z2=0.0f;
}

static inline float bq_run(BQ *q, float x){
  float y = q->b0*x + q->z1;
  q->z1 = q->b1*x + q->z2 - q->a1*y;
  q->z2 = q->b2*x - q->a2*y;
  return y;
}

// Коэфы: bilinear Fs=44100, Fc=4800, Q=0.7071 (предварительно посчитано)
static void bq_lowpass_4k8(BQ *q){
  // a0=1
  const float b0=0.06745527f, b1=0.13491054f, b2=0.06745527f;
  const float a1=-1.1429805f, a2=0.41280160f;
  bq_init(q,b0,b1,b2,a1,a2);
}

int main(int argc, char**argv){
  if (argc < 5){
    fprintf(stderr,"Usage:\n  %s <wav_file> <spidev> <A|B> <spi_hz> [volume]\n", argv[0]);
    return 1;
  }
  const char* wavp=argv[1];
  const char* spidev=argv[2];
  int chB=(argv[3][0]=='B'||argv[3][0]=='b');
  uint32_t spi_hz=(uint32_t)strtoul(argv[4],NULL,10);
  float vol = (argc>=6)? strtof(argv[5],NULL) : 1.0f;
  if (vol < 0.01f) vol = 0.01f;
  if (vol > 2.0f)  vol = 2.0f;

  // WAV
  int wfd=open(wavp,O_RDONLY); if(wfd<0){perror("open wav"); return 1;}
  struct stat st; if(fstat(wfd,&st)!=0){perror("fstat"); close(wfd); return 1;}
  WavHdr h;
  if (read(wfd,&h,sizeof(h))!=(ssize_t)sizeof(h)){ fprintf(stderr,"Bad WAV header\n"); close(wfd); return 1; }
  if (h.chunk_id!=0x46464952 || h.format!=0x45564157 || h.sub1_id!=0x20746d66){ fprintf(stderr,"Not RIFF/WAVE/PCM fmt\n"); close(wfd); return 1; }
  if (h.audio_format!=1 || h.bits_per_sample!=16){ fprintf(stderr,"Need PCM 16-bit\n"); close(wfd); return 1; }
  if (h.sub1_size>16){ if (lseek(wfd, h.sub1_size-16, SEEK_CUR)<0){ perror("lseek fmt"); close(wfd); return 1; } }

  uint32_t data_sz=0;
  if (find_data(wfd,&data_sz)!=0){ fprintf(stderr,"No data chunk\n"); close(wfd); return 1; }
  off_t cur=lseek(wfd,0,SEEK_CUR); off_t tail=(cur>=0 && st.st_size>cur)? (st.st_size-cur):0;
  if ((off_t)data_sz>tail){
    fprintf(stderr,"WAV warns: header data_size=%u > tail=%jd, clamping.\n", data_sz, (intmax_t)tail);
    data_sz=(uint32_t)tail;
  }

  uint32_t sr=h.sample_rate; uint16_t chs=h.num_channels; uint16_t blk=h.block_align;
  if (sr != 44100){
    fprintf(stderr,"Note: this player expects 44100 Hz input; got %u Hz.\n", sr);
  }
  size_t frames_total=data_sz/blk;
  int16_t* pcm=(int16_t*)malloc(frames_total*chs*sizeof(int16_t));
  if(!pcm){perror("malloc"); close(wfd); return 1;}
  if (read(wfd, pcm, frames_total*chs*2)!=(ssize_t)(frames_total*chs*2)){ perror("read pcm"); free(pcm); close(wfd); return 1; }
  close(wfd);
  fprintf(stderr,"WAV in: %u Hz, 16-bit, %u ch, frames=%zu (%.2f s)\n",
          sr, chs, frames_total, frames_total/(double)sr);

  // SPI
  uint32_t spi_hz_drv=spi_hz;
  int sfd=open_spi(spidev,&spi_hz_drv); if(sfd<0){ free(pcm); return 1; }
  fprintf(stderr,"SPI speed set: %u Hz\n", spi_hz_drv);

  // Реал-тайм + привязка к CPU0 (если получится)
  cpu_set_t cpus; CPU_ZERO(&cpus); CPU_SET(0, &cpus); sched_setaffinity(0, sizeof(cpus), &cpus);
  struct sched_param sp = {.sched_priority=50}; sched_setscheduler(0, SCHED_FIFO, &sp);

  // Целевая частота вывода (фиксированная)
  const uint32_t Fs_out = 11025; // 44100/4
  const uint64_t period_ns = (uint64_t) llround(1000000000.0 / (double)Fs_out);

  // Подготовим би-квад ФНЧ и децимацию ×4
  BQ lp; bq_lowpass_4k8(&lp);
  const int DECIM = 4;

  // SPI transfer (1 семпл = 1 ioctl)
  uint8_t tx[2];
  struct spi_ioc_transfer tr; memset(&tr,0,sizeof(tr));
  tr.tx_buf        = (uint64_t)(uintptr_t)tx;
  tr.len           = 2;
  tr.speed_hz      = spi_hz_drv;
  tr.bits_per_word = 8;
  tr.cs_change     = 0;
  tr.delay_usecs   = 0;

  // Основной цикл: фильтр -> децимация -> дизер -> квантование -> SPI
  uint64_t t_next = now_ns(); // старт немедленно
  size_t out_frames = 0;
  uint32_t rng = 0x12345678u;

  size_t i=0;
  float acc = 0.0f; int dec = 0;
  while (i < frames_total){
    // Возьмём очередной входной сэмпл (левый канал)
    int16_t s = (chs==1)? pcm[i] : pcm[i*chs + 0];
    i++;

    // Нормализуем во float и прогоняем через ФНЧ
    float x = (float)s / 32768.0f;
    float y = bq_run(&lp, x);

    // На каждый 4-й отфильтрованный сэмпл — вывод
    if (++dec == DECIM){
      dec = 0;
      // Блокируем току постоянную (микродрейф) — простая защита
      // (по желанию: lp.z1/z2 уже держат DC, можно без HPF)
      float yf = y;

      // Назначаем время следующего сэмпла
      t_next += period_ns;

      // Ждём до t_next (абсолютно), просыпаемся чуть раньше
      uint64_t now = now_ns();
      if (now + 5000ull < t_next){
        struct timespec ts; ts_from_ns(t_next - 3000ull, &ts);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
      }
      while ((now = now_ns()) < t_next) { /* короткий spin */ }

      // Квантование -> 12 бит с TPDF-дизером
      int16_t s16 = (int16_t)CLAMP((int)lrintf(yf * 32767.0f), -32768, 32767);
      uint16_t u12 = s16_to_u12_dither(s16, vol, &rng);
      uint16_t w   = mcp4822_word(chB, u12);
      tx[0] = (uint8_t)(w>>8); tx[1] = (uint8_t)w;

      if (ioctl(sfd, SPI_IOC_MESSAGE(1), &tr) < 0){
        perror("SPI_IOC_MESSAGE(1)"); break;
      }
      out_frames++;
    }
  }

  uint64_t t_done = now_ns();
  double wall = (t_done - (t_next - out_frames*period_ns))/1e9;
  fprintf(stderr,"Done. OutFrames=%zu, wall≈%.3f s, expected=%.3f s @ %u Hz.\n",
          out_frames, wall, out_frames/(double)Fs_out, Fs_out);

  close(sfd); free(pcm);
  return 0;
}
