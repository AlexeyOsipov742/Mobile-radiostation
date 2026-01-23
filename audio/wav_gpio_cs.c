// wav_gpio_cs_napi.c — WAV -> MCP4822 через spidev, CS управляется GPIO (libgpiod)
// Основано на "старом правильном" wav.c: streaming, WAVE_FORMAT_EXTENSIBLE, FIR+DECIM, fade, abs timing.
// CS: по умолчанию /dev/gpiochip2 line 4 (GPIO2_A4 = header1-pin7 на NaPi).
//
// Сборка:
//   gcc -O2 -Wall -Wextra -std=gnu11 -o wav_gpio_cs_napi wav_gpio_cs_napi.c -lm -lgpiod
//
// Запуск:
//   sudo ./wav_gpio_cs_napi <wav> <spidev> <A|B> <spi_hz> [volume]
//   sudo ./wav_gpio_cs_napi mic.wav /dev/spidev2.0 A 800000 1.0
//
// Опционально:
//   --cs-chip /dev/gpiochip2   --cs-line 4     (если захочешь изменить)

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
#include <gpiod.h>

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
static inline void ts_from_ns(uint64_t tns, struct timespec*ts){
  ts->tv_sec=(time_t)(tns/1000000000ull);
  ts->tv_nsec=(long)(tns%1000000000ull);
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

static int open_spi_no_cs(const char*dev, uint32_t *hz_inout){
  int fd = open(dev, O_RDWR);
  if (fd < 0){ perror("open spidev"); return -1; }

  uint8_t mode = (uint8_t)(SPI_MODE_0);
  uint8_t bpw = 8;

  if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1){ perror("SPI_IOC_WR_MODE"); goto fail; }
  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) == -1){ perror("SPI_IOC_WR_BITS_PER_WORD"); goto fail; }

  uint32_t hz=*hz_inout; if (hz==0) hz=1000000u;
  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) == -1){ perror("SPI_IOC_WR_MAX_SPEED_HZ"); goto fail; }
  uint32_t rd=0; if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &rd) == -1) rd=hz;
  if (rd && rd!=*hz_inout) fprintf(stderr,"WARN: requested %u Hz, driver %u Hz\n", *hz_inout, rd);
  if (rd) *hz_inout = rd;
  return fd;

fail:
  close(fd); return -1;
}

// MCP4822: [15]=B, [14]=BUF=0, [13]=GA=1, [12]=SHDN=1, [11:0]=DATA
static inline uint16_t mcp4822_word(int chB, uint16_t u12){
  return (uint16_t)(((chB?1:0)<<15) | (1<<13) | (1<<12) | (u12 & 0x0FFF));
}

// 16->12 с TPDF-дизером (для настоящих 16 бит)
static inline uint16_t s16_to_u12_dither(int16_t s, float vol, uint32_t *rng){
  float x = (float)s / 32768.f; x *= vol;
  if (x > 0.999f) x = 0.999f;
  else if (x < -0.999f) x = -0.999f;

  *rng = *rng*1664525u + 1013904223u; float u1=(((*rng>>8)&0xFFFF)/65535.0f);
  *rng = *rng*1664525u + 1013904223u; float u2=(((*rng>>8)&0xFFFF)/65535.0f);
  float d=(u1+u2-1.0f)/4095.0f; // 1 LSB @ 12-bit
  float y=(x*0.5f + 0.5f) + d;

  int v=(int)lrintf(y*4095.f);
  if (v<0) v=0; else if (v>4095) v=4095;
  return (uint16_t)v;
}

/* FIR (окно Хэмминга) */
static void fir_lowpass_hamming(float *h, int N, float fc){
  int M = N - 1; double sum = 0.0;
  for (int n=0; n<N; ++n){
    int k = n - M/2;
    double sinc = (k==0) ? 1.0 : sin(M_PI * 2.0 * fc * k) / (M_PI * k);
    double w = 0.54 - 0.46 * cos(2.0 * M_PI * n / M);
    double v = (2.0 * fc) * sinc * w; h[n] = (float)v; sum += v;
  }
  if (sum != 0.0){
    double g = 1.0 / sum;
    for (int n=0; n<N; ++n) h[n] = (float)(h[n] * g);
  }
}

static int arg_after(int argc, char**argv, const char* key, char* out, size_t out_sz){
  for(int i=1;i<argc-1;i++){
    if(strcmp(argv[i], key)==0){
      snprintf(out, out_sz, "%s", argv[i+1]);
      return 1;
    }
  }
  return 0;
}

int main(int argc, char**argv){
  if (argc < 5){
    fprintf(stderr,
      "Usage:\n"
      "  %s <wav> <spidev> <A|B> <spi_hz> [volume] [--cs-chip /dev/gpiochip2 --cs-line 4]\n",
      argv[0]);
    return 1;
  }

  const char* wavp   = argv[1];
  const char* spidev = argv[2];
  int chB            = (argv[3][0]=='B'||argv[3][0]=='b');
  uint32_t spi_hz    = (uint32_t)strtoul(argv[4],NULL,10);
  float vol          = (argc>=6)? strtof(argv[5],NULL) : 1.0f;
  if (vol < 0.01f) vol = 0.01f;
  if (vol > 2.0f)  vol = 2.0f;

  // CS GPIO defaults for NaPi
  const char* cs_chip_path = "/dev/gpiochip2";
  unsigned cs_line_offset  = 4;

  char tmp[128];
  if (arg_after(argc, argv, "--cs-chip", tmp, sizeof(tmp))) cs_chip_path = strdup(tmp);
  if (arg_after(argc, argv, "--cs-line", tmp, sizeof(tmp))) cs_line_offset = (unsigned)strtoul(tmp, NULL, 10);

  // WAV open + parse
  int wfd=open(wavp,O_RDONLY);
  if(wfd<0){ perror("open wav"); return 1; }

  struct stat st;
  if(fstat(wfd,&st)!=0){ perror("fstat"); close(wfd); return 1; }

  WavHdr h;
  if (read(wfd,&h,sizeof(h))!=(ssize_t)sizeof(h)){
    fprintf(stderr,"Bad WAV header\n");
    close(wfd);
    return 1;
  }
  if (h.chunk_id!=0x46464952 || h.format!=0x45564157 || h.sub1_id!=0x20746d66){
    fprintf(stderr,"Not RIFF/WAVE 'fmt '\n");
    close(wfd);
    return 1;
  }

  uint16_t fmt_tag = h.audio_format;
  uint16_t chs     = h.num_channels;
  uint32_t sr      = h.sample_rate;
  uint16_t bits    = h.bits_per_sample;
  uint16_t valid_bits = bits;

  if (h.sub1_size>16){
    uint16_t cb;
    if (read(wfd,&cb,2)!=2){ perror("read cbSize"); close(wfd); return 1; }
    if (fmt_tag==0xFFFE && cb>=22){
      if (read(wfd,&valid_bits,2)!=2){ perror("read valid_bits"); close(wfd); return 1; }
      if (lseek(wfd, 4+16, SEEK_CUR) < 0){ perror("lseek ext"); close(wfd); return 1; }
      int extra = (int)cb - 22;
      if (extra>0 && lseek(wfd, extra, SEEK_CUR) < 0){ perror("lseek extra"); close(wfd); return 1; }
    } else {
      if (lseek(wfd, (off_t)h.sub1_size-16, SEEK_CUR) < 0){ perror("lseek fmt extra"); close(wfd); return 1; }
    }
  }

  uint32_t data_sz=0;
  if (find_data(wfd,&data_sz)!=0){ fprintf(stderr,"No data chunk\n"); close(wfd); return 1; }

  // clamp data_sz to file tail (защита от кривого заголовка)
  off_t cur = lseek(wfd, 0, SEEK_CUR);
  off_t tail = (cur>=0 && st.st_size>cur) ? (st.st_size - cur) : 0;
  if ((off_t)data_sz > tail){
    fprintf(stderr,"WAV warn: header data=%u > tail=%jd, clamping.\n", data_sz, (intmax_t)tail);
    data_sz = (uint32_t)tail;
  }

  if (bits!=16){ fprintf(stderr,"Need 16-bit container WAV (got %u)\n", bits); close(wfd); return 1; }
  if (chs<1){ fprintf(stderr,"Need at least 1 channel\n"); close(wfd); return 1; }

  fprintf(stderr,"WAV in: %u Hz, %u-bit container, valid=%u, %u ch, data=%u bytes\n",
          sr, bits, valid_bits, chs, data_sz);

  // SPI (NO_CS)
  uint32_t spi_hz_drv=spi_hz;
  int sfd=open_spi_no_cs(spidev,&spi_hz_drv);
  if(sfd<0){ close(wfd); return 1; }
  fprintf(stderr,"SPI opened: %s, mode=0 +NO_CS, speed=%u Hz\n", spidev, spi_hz_drv);

  // GPIO CS (libgpiod)
  struct gpiod_chip *chip = gpiod_chip_open(cs_chip_path);
  if (!chip){
    fprintf(stderr,"gpiod_chip_open(%s) failed: %s\n", cs_chip_path, strerror(errno));
    close(sfd); close(wfd);
    return 1;
  }
  struct gpiod_line *csline = gpiod_chip_get_line(chip, (unsigned)cs_line_offset);
  if (!csline){
    fprintf(stderr,"gpiod_chip_get_line(%s,%u) failed: %s\n", cs_chip_path, cs_line_offset, strerror(errno));
    gpiod_chip_close(chip);
    close(sfd); close(wfd);
    return 1;
  }
  if (gpiod_line_request_output(csline, "wav-gpio-cs", 1) != 0){
    fprintf(stderr,"gpiod_line_request_output failed: %s\n", strerror(errno));
    gpiod_chip_close(chip);
    close(sfd); close(wfd);
    return 1;
  }
  // idle high
  (void)gpiod_line_set_value(csline, 1);

  fprintf(stderr,"CS GPIO: chip=%s line=%u (idle HIGH)\n", cs_chip_path, cs_line_offset);

  // Реал-тайм (best-effort)
  {
    cpu_set_t cpus; CPU_ZERO(&cpus); CPU_SET(0,&cpus);
    (void)sched_setaffinity(0,sizeof(cpus),&cpus);
    struct sched_param sp = {.sched_priority=50};
    if (sched_setscheduler(0,SCHED_FIFO,&sp) != 0){
      fprintf(stderr,"WARN: sched_setscheduler(SCHED_FIFO) failed: %s\n", strerror(errno));
    }
  }

  // Авто-DECIM к 11025 (как в старом wav.c)
  int DECIM = (int)lround((double)sr / 11025.0);
  if (DECIM < 1) DECIM = 1;
  const double Fs_out = (double)sr / (double)DECIM;
  const uint64_t period_ns = (uint64_t) llround(1e9 / Fs_out);
  fprintf(stderr,"Output Fs ≈ %.2f Hz (DECIM=%d)\n", Fs_out, DECIM);

  // FIR
  enum { NTAP = 63 };
  float fir[NTAP]; memset(fir,0,sizeof(fir));
  fir_lowpass_hamming(fir, NTAP, 0.22f);

  // Кольцевой буфер
  float rb[NTAP]; memset(rb,0,sizeof(rb));
  int rpos=0;
  int dec=0;
  uint32_t rng=0x12345678u;

  // SPI transfer (1 sample -> 2 bytes)
  uint8_t txb[2];
  struct spi_ioc_transfer tr;
  memset(&tr,0,sizeof(tr));
  tr.tx_buf        = (uint64_t)(uintptr_t)txb;
  tr.len           = 2;
  tr.speed_hz      = spi_hz_drv;
  tr.bits_per_word = 8;
  tr.cs_change     = 0;
  tr.delay_usecs   = 0;

  // Fade-in/out ~5 мс
  const int fade_out_len = (int)(0.005 * Fs_out + 0.5);
  const int fade_in_len  = fade_out_len;

  // Стриминговое чтение
  const size_t FRAMES_PER_CHUNK = 4096;
  const size_t BYTES_PER_FRAME  = (size_t)chs * 2; // 16-bit container
  uint8_t *buf = (uint8_t*)malloc(FRAMES_PER_CHUNK * BYTES_PER_FRAME);
  if (!buf){
    perror("malloc chunk");
    gpiod_line_release(csline);
    gpiod_chip_close(chip);
    close(sfd); close(wfd);
    return 1;
  }

  uint64_t t_next = now_ns();
  size_t out_frames=0;
  int use_raw12 = (valid_bits <= 12);
  uint32_t remaining = data_sz;

  // main loop
  while (remaining > 0){
    size_t toread = FRAMES_PER_CHUNK * BYTES_PER_FRAME;
    if (toread > remaining) toread = remaining;

    ssize_t got_bytes = read(wfd, buf, toread);
    if (got_bytes <= 0){
      if (got_bytes < 0) perror("read data");
      break;
    }
    remaining -= (uint32_t)got_bytes;

    int16_t *s16p = (int16_t*)buf;
    size_t frames = (size_t)got_bytes / BYTES_PER_FRAME;

    for (size_t i=0;i<frames;i++){
      int16_t s = (chs==1)? s16p[i] : s16p[i*chs + 0];
      float x = (float)s / 32768.0f;

      if (--rpos < 0) rpos = NTAP-1;
      rb[rpos] = x;

      if (++dec == DECIM){
        dec = 0;

        float acc=0.0f;
        int idx=rpos;
        for (int k=0; k<NTAP; ++k){
          acc += fir[k] * rb[idx++];
          if (idx==NTAP) idx=0;
        }

        int frame_index = (int)out_frames;
        float in_left_frames = (float)remaining / (float)BYTES_PER_FRAME;
        int remain_out = (int)((in_left_frames + (frames - i - 1)) / DECIM);

        float gain_env = 1.0f;
        if (frame_index < fade_in_len) gain_env *= (frame_index / (float)fade_in_len);
        if (remain_out < fade_out_len) gain_env *= (remain_out / (float)fade_out_len);
        if (gain_env < 0.0f) gain_env = 0.0f;
        else if (gain_env > 1.0f) gain_env = 1.0f;

        acc *= gain_env;

        // Абсолютная синхронизация вывода
        t_next += period_ns;
        uint64_t now = now_ns();
        if (now + 5000ull < t_next){
          struct timespec ts; ts_from_ns(t_next - 3000ull, &ts);
          clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        }
        while ((now = now_ns()) < t_next) { /* spin */ }

        // 12-бит код
        uint16_t u12;
        if (use_raw12){
          int16_t y = (int16_t)CLAMP((int)lrintf(acc * 32767.0f), -32768, 32767);
          int32_t code_signed12 = y >> 4;           // -2048..+2047
          int32_t u = code_signed12 + 2048;         // 0..4095
          if (u<0) u=0; else if (u>4095) u=4095;
          u12 = (uint16_t)u;
        } else {
          int16_t y = (int16_t)CLAMP((int)lrintf(acc * 32767.0f), -32768, 32767);
          u12 = s16_to_u12_dither(y, vol, &rng);
        }

        uint16_t w = mcp4822_word(chB, u12);
        txb[0]=(uint8_t)(w>>8);
        txb[1]=(uint8_t)w;

        // GPIO-CS around the SPI frame
        (void)gpiod_line_set_value(csline, 0);
        if (ioctl(sfd, SPI_IOC_MESSAGE(1), &tr) < 0){
          perror("SPI_IOC_MESSAGE(1)");
          (void)gpiod_line_set_value(csline, 1);
          free(buf);
          gpiod_line_release(csline);
          gpiod_chip_close(chip);
          close(sfd);
          close(wfd);
          return 1;
        }
        (void)gpiod_line_set_value(csline, 1);

        ++out_frames;
      }
    }
  }

  {
    uint64_t t_done = now_ns();
    double wall = (t_done - (t_next - (uint64_t)llround(out_frames*(1e9/Fs_out))))/1e9;
    fprintf(stderr,"Done. OutFrames=%zu, wall≈%.3f s, expected=%.3f s @ %.2f Hz.\n",
            out_frames, wall, out_frames/Fs_out, Fs_out);
  }

  free(buf);
  gpiod_line_release(csline);
  gpiod_chip_close(chip);
  close(sfd);
  close(wfd);
  return 0;
}

