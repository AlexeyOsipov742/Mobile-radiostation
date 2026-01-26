// mic.c — запись MCP3201 -> WAV (mono 16-bit) через spidev, с учетом ограничений SPI_IOC_MESSAGE
//
// Главное отличие от твоей версии:
// 1) max_xfers по умолчанию 128 и ЖЕСТКО ограничивается безопасным лимитом (страница/ioctl size)
// 2) проверяется ret от ioctl: должен быть == 2*n (байт)
// 3) запись в WAV буферизована (fwrite() одним блоком), а не по сэмплу
// 4) опционально можно "выкинуть/заменить" первый сэмпл каждого ioctl (--drop-first)
//    (часто лечит одиночный битый сэмпл на границе сообщений)
//
// Сборка:
//   gcc -O2 -Wall -Wextra -std=gnu11 -o mic mic.c -lm
//
// Пример запуска (Pi1 обычно реально потянет 44.1k только при нормальном SPI/железе и буферной записи):
//   sudo ./mic /dev/spidev0.0 out.wav 44100 1.0 1024 0 705600 --autosr --meter
//
// Если видишь "Message too long" — уменьши --max-xfers (на Pi чаще всего максимум 128).
//

#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

/* --------- util --------- */
static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig){ (void)sig; g_stop = 1; }

static uint64_t now_ns(void){
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec;
}

static struct timespec ns_to_ts(uint64_t ns){
  struct timespec ts;
  ts.tv_sec  = (time_t)(ns / 1000000000ull);
  ts.tv_nsec = (long)(ns % 1000000000ull);
  return ts;
}

static void sleep_until_ns(uint64_t deadline_ns){
  struct timespec ts = ns_to_ts(deadline_ns);
  while(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) != 0){
    if(errno == EINTR) continue;
    break;
  }
}

static int has_flag(int argc, char**argv, const char* flag){
  for(int i=1;i<argc;i++) if(strcmp(argv[i],flag)==0) return 1;
  return 0;
}

static int get_arg_after(int argc, char**argv, const char* key, char* out, size_t out_sz){
  for(int i=1;i<argc-1;i++){
    if(strcmp(argv[i], key)==0){
      snprintf(out, out_sz, "%s", argv[i+1]);
      return 1;
    }
  }
  return 0;
}

/* --------- simple 1st-order DC blocker (HPF) --------- */
/* y[n] = x[n] - x[n-1] + R * y[n-1], R = exp(-2*pi*fc/fs) */
typedef struct { float R, x1, y1; } HPF1;
static void hpf1_init(HPF1* h, float fs, float fc){
  float R = expf(-2.0f*(float)M_PI*fc/fs);
  if(R < 0.0f) R = 0.0f;
  if(R > 0.9999f) R = 0.9999f;
  h->R = R; h->x1 = 0.0f; h->y1 = 0.0f;
}
static inline float hpf1_run(HPF1* h, float x){
  float y = x - h->x1 + h->R*h->y1;
  h->x1 = x; h->y1 = y; return y;
}

/* --------- WAV helpers (mono, 16-bit) --------- */
static void put_le32(uint8_t* p, uint32_t v){
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static int write_wav_header(FILE* f, uint32_t sr){
  uint8_t hdr[44]; memset(hdr, 0, sizeof(hdr));
  memcpy(hdr+0,  "RIFF", 4);
  memcpy(hdr+8,  "WAVE", 4);
  memcpy(hdr+12, "fmt ", 4);

  put_le32(hdr+16, 16);      /* fmt chunk size */
  hdr[20]=1; hdr[21]=0;      /* PCM */
  hdr[22]=1; hdr[23]=0;      /* mono */

  put_le32(hdr+24, sr);      /* sample rate */

  uint32_t br = sr * 2u;     /* byte rate (mono, 16-bit) */
  put_le32(hdr+28, br);

  hdr[32]=2; hdr[33]=0;      /* block align */
  hdr[34]=16; hdr[35]=0;     /* bits */

  memcpy(hdr+36,"data",4);

  return (fwrite(hdr,1,sizeof(hdr),f)==sizeof(hdr))?0:-1;
}

/* Patch only the SR and byte-rate fields in the existing header. */
static int patch_wav_samplerate(FILE* f, uint32_t sr){
  /* sample rate @ offset 24, byte rate @ offset 28 */
  if(fseek(f, 24, SEEK_SET) != 0) return -1;
  uint8_t tmp[8];
  put_le32(tmp+0, sr);
  put_le32(tmp+4, sr*2u);
  if(fwrite(tmp, 1, sizeof(tmp), f) != sizeof(tmp)) return -1;
  return 0;
}

static int finalize_wav_sizes(FILE* f){
  long file_size = ftell(f);
  if(file_size < 44) return -1;
  uint32_t data_size = (uint32_t)(file_size - 44);
  uint32_t riff_size = data_size + 36;

  if(fseek(f, 4, SEEK_SET)!=0) return -1;
  uint8_t tmp[4];
  put_le32(tmp, riff_size);
  if(fwrite(tmp,1,4,f)!=4) return -1;

  if(fseek(f, 40, SEEK_SET)!=0) return -1;
  put_le32(tmp, data_size);
  if(fwrite(tmp,1,4,f)!=4) return -1;

  return 0;
}

/* --------- SPI open --------- */
static int open_spi(const char* dev, uint32_t* p_hz, uint8_t mode, int* out_fd){
  int fd = open(dev, O_RDWR);
  if(fd<0){ perror("open spidev"); return -1; }

  uint8_t bpw = 8;
  if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) { perror("SPI_IOC_WR_MODE"); close(fd); return -1; }
  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0) { perror("SPI_IOC_WR_BITS_PER_WORD"); close(fd); return -1; }

  uint32_t hz = *p_hz;
  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) { perror("SPI_IOC_WR_MAX_SPEED_HZ"); close(fd); return -1; }
  ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &hz);
  *p_hz = hz;
  *out_fd = fd;

  fprintf(stderr, "SPI opened: mode=%u %s, %ub, speed=%u Hz\n",
          (unsigned)(mode & 3),
          (mode & SPI_CS_HIGH) ? " +CS_HIGH" : " +CS_LOW",
          (unsigned)bpw, hz);
  return 0;
}

/* --------- MCP3201 parse (2 bytes) ---------
   Typically: first byte contains [xxxx 0 B11 B10 B9 B8 B7]
             second byte contains [B6 B5 B4 B3 B2 B1 B0 x]
*/
static inline uint16_t parse_u12_mcp3201(const uint8_t* r2){
  uint16_t upper5 = (uint16_t)(r2[0] & 0x1F);
  uint16_t lower7 = (uint16_t)((r2[1] >> 1) & 0x7F);
  return (uint16_t)((upper5 << 7) | lower7);
}

/* Compute a "safe" max_xfers limit for SPI_IOC_MESSAGE.
   There are two practical ceilings:
   - ioctl size field: max 16383 bytes for the argument
   - many spidev builds effectively limit to one page (~4096 bytes) for the ioc_transfer array
*/
static size_t compute_safe_max_xfers(void){
  size_t sz = sizeof(struct spi_ioc_transfer);
  size_t lim_ioctl = 16383u / sz;   /* max encoded size */
  size_t lim_page  = 4096u  / sz;   /* common spidev limit */
  size_t lim = lim_ioctl;
  if(lim_page > 0 && lim_page < lim) lim = lim_page;
  if(lim < 1) lim = 1;
  return lim;
}

/* One ioctl reading n samples; each sample = 2 bytes (16 clocks).
   CS toggled between samples via cs_change=1 and delay_usecs (CS high time). */
static int spi_read_samples_mcp3201(
    int fd, uint32_t spi_hz,
    uint8_t* tx, uint8_t* rx,
    struct spi_ioc_transfer* trs,
    size_t n, uint16_t tCSH_us)
{
  memset(tx, 0, n*2);
  memset(rx, 0, n*2);
  memset(trs, 0, n*sizeof(*trs));

  for(size_t i=0;i<n;i++){
    trs[i].tx_buf = (unsigned long)(tx + 2*i);
    trs[i].rx_buf = (unsigned long)(rx + 2*i);
    trs[i].len = 2;
    trs[i].speed_hz = spi_hz;
    trs[i].bits_per_word = 8;
    trs[i].cs_change = 1;
    trs[i].delay_usecs = tCSH_us; /* CS high pause between samples */
  }

  int ret = ioctl(fd, SPI_IOC_MESSAGE((int)n), trs);
  if(ret < 0) return -1;

  /* IMPORTANT: should transfer exactly 2*n bytes */
  if(ret != (int)(2*n)){
    errno = EMSGSIZE;
    return -1;
  }
  return 0;
}

int main(int argc, char**argv){
  if(argc < 3){
    fprintf(stderr,
      "Usage:\n"
      "  %s <spidev> <out.wav> [sr=16000] [gain=1.0] [chunk=1024] [mode=0|3]\n"
      "     [spi_hz=1000000] [--cs-high] [--max-xfers N] [--meter] [--pace] [--autosr]\n"
      "     [--drop-first]\n"
      "\n"
      "Flags:\n"
      "  --pace      : keep real sampling rate close to sr (stable playback speed)\n"
      "  --autosr    : after recording, write real Eff.fs into WAV header sample-rate\n"
      "  --max-xfers : transfers per ioctl (Pi often max 128). Larger may fail/overflow.\n"
      "  --meter     : print min/max raw ADC code each second\n"
      "  --drop-first: replace first sample of each ioctl by the second (helps boundary glitch)\n",
      argv[0]);
    return 1;
  }

  const char* spidev = argv[1];
  const char* outwav = argv[2];

  uint32_t sr   = (argc>=4)? (uint32_t)strtoul(argv[3],NULL,10) : 16000u;
  float gain    = (argc>=5)? strtof(argv[4],NULL) : 1.0f;
  if(gain < 0.1f) gain = 0.1f;
  if(gain > 16.0f) gain = 16.0f;

  size_t CHUNK  = (argc>=6)? (size_t)strtoul(argv[5],NULL,10) : 1024u;
  if(CHUNK < 64) CHUNK = 64;
  if(CHUNK > 16384) CHUNK = 16384;

  uint8_t base_mode = (argc>=7 && argv[6][0]=='3') ? SPI_MODE_3 : SPI_MODE_0;
  uint32_t spi_hz   = (argc>=8)? (uint32_t)strtoul(argv[7],NULL,10) : 1000000u;

  int meter     = has_flag(argc,argv,"--meter");
  int want_high = has_flag(argc,argv,"--cs-high");
  int pace      = has_flag(argc,argv,"--pace");
  int autosr    = has_flag(argc,argv,"--autosr");
  int drop_first= has_flag(argc,argv,"--drop-first");

  uint8_t mode  = want_high ? (base_mode | SPI_CS_HIGH) : (base_mode & ~SPI_CS_HIGH);

  size_t safe_lim = compute_safe_max_xfers();
  size_t max_xfers = 128; /* good default on Pi */
  char tmp[64];
  if(get_arg_after(argc, argv, "--max-xfers", tmp, sizeof(tmp))){
    max_xfers = (size_t)strtoul(tmp, NULL, 10);
    if(max_xfers < 1) max_xfers = 1;
  }
  if(max_xfers > safe_lim){
    fprintf(stderr, "WARN: --max-xfers %zu too high for this system; clamping to %zu (safe limit)\n",
            max_xfers, safe_lim);
    max_xfers = safe_lim;
  }

  int sfd;
  if(open_spi(spidev, &spi_hz, mode, &sfd) != 0) return 1;

  FILE* f = fopen(outwav, "wb");
  if(!f){ perror("fopen wav"); close(sfd); return 1; }
  if(write_wav_header(f, sr) != 0){
    fprintf(stderr,"write_wav_header failed\n");
    fclose(f); close(sfd); return 1;
  }

  uint8_t* tx = (uint8_t*)malloc(max_xfers*2);
  uint8_t* rx = (uint8_t*)malloc(max_xfers*2);
  struct spi_ioc_transfer* trs = (struct spi_ioc_transfer*)malloc(max_xfers*sizeof(*trs));
  int16_t* pcm_block = (int16_t*)malloc(max_xfers*sizeof(int16_t));
  if(!tx || !rx || !trs || !pcm_block){
    perror("malloc");
    fclose(f); close(sfd);
    free(tx); free(rx); free(trs); free(pcm_block);
    return 1;
  }

  struct sigaction sa; memset(&sa,0,sizeof(sa));
  sa.sa_handler = on_sigint;
  sigaction(SIGINT, &sa, NULL);

  HPF1 hpf; hpf1_init(&hpf, (float)sr, 20.0f);

  size_t total = 0;
  uint64_t t_start = now_ns();

  /* pacing by CHUNK */
  const uint64_t block_ns = (uint64_t)((1000000000.0 * (double)CHUNK) / (double)sr);
  uint64_t next_deadline = t_start + block_ns;

  /* meter */
  uint64_t t0 = t_start;
  uint16_t minv=0xFFFF, maxv=0;

  fprintf(stderr,
    "Recording… MCP3201, sr=%u Hz, chunk=%zu, max_xfers=%zu (safe_lim=%zu), spi_hz=%u, pace=%s, autosr=%s, drop_first=%s. Ctrl+C to stop.\n",
    sr, CHUNK, max_xfers, safe_lim, spi_hz,
    pace ? "ON" : "OFF",
    autosr ? "ON" : "OFF",
    drop_first ? "ON" : "OFF");

  const uint16_t tCSH_us = 1;

  while(!g_stop){
    size_t remaining = CHUNK;

    while(remaining && !g_stop){
      size_t n = remaining;
      if(n > max_xfers) n = max_xfers;

      if(spi_read_samples_mcp3201(sfd, spi_hz, tx, rx, trs, n, tCSH_us) != 0){
        perror("SPI_IOC_MESSAGE");
        g_stop = 1;
        break;
      }

      /* Optional: fix first-sample glitch per ioctl */
      if(drop_first && n >= 2){
        rx[0] = rx[2];
        rx[1] = rx[3];
      }

      for(size_t i=0;i<n;i++){
        uint16_t u12 = parse_u12_mcp3201(rx + 2*i);

        if(meter){
          if(u12 < minv) minv = u12;
          if(u12 > maxv) maxv = u12;
        }

        float s = ((int)u12 - 2048) * (1.0f/2048.0f);
        s = hpf1_run(&hpf, s) * gain;

        if(s > 0.999f) s = 0.999f;
        if(s < -0.999f) s = -0.999f;

        pcm_block[i] = (int16_t)lrintf(s * 32767.0f);
      }

      if(fwrite(pcm_block, sizeof(int16_t), n, f) != n){
        perror("fwrite");
        g_stop = 1;
        break;
      }

      total += n;
      remaining -= n;
    }

    if(pace){
      uint64_t now = now_ns();
      if(now < next_deadline){
        sleep_until_ns(next_deadline);
      }
      next_deadline += block_ns;

      /* if too late, resync to avoid infinite lag growth */
      uint64_t late = now_ns();
      if(late > next_deadline + 5*block_ns){
        next_deadline = late + block_ns;
      }
    }

    if(meter){
      uint64_t now = now_ns();
      if(now - t0 >= 1000000000ull){
        fprintf(stderr,"METER: raw_u12[min=%4u max=%4u mid≈2048], wrote=%zu\n",
                minv, maxv, total);
        minv=0xFFFF; maxv=0; t0=now;
      }
    }
  }

  /* compute real eff.fs BEFORE patching header */
  uint64_t t_end = now_ns();
  double wall = (t_end - t_start)/1e9;
  double eff_sr = total / (wall>0? wall : 1.0);

  if(finalize_wav_sizes(f)!=0) fprintf(stderr,"WARN: finalize_wav_sizes failed\n");

  if(autosr){
    uint32_t eff_u = (uint32_t)lround(eff_sr);
    if(eff_u < 1000) eff_u = 1000;
    if(eff_u > 300000) eff_u = 300000;
    if(patch_wav_samplerate(f, eff_u)!=0){
      fprintf(stderr,"WARN: patch_wav_samplerate failed\n");
    }else{
      fprintf(stderr,"AUTOSR: wrote sample_rate=%u Hz into WAV header\n", eff_u);
    }
  }

  fclose(f);
  close(sfd);
  free(tx); free(rx); free(trs); free(pcm_block);

  fprintf(stderr,"Done. Wrote %zu samples (%.2f s @ header_sr=%u). Eff.fs≈%.1f Hz\n",
          total, total/(double)sr, sr, eff_sr);

  return 0;
}

