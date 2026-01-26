// ptt_audio.c — PTT "радиостанция": TX=запись с MCP3201 (SPI ADC) -> RAM,
// RX=воспроизведение из RAM -> MCP4822 (SPI DAC) по логике "старого корректного wav.c".
// GPIO PTT читается через libgpiod.
//
// Важно: playback использует логику старого wav.c:
//   - Fs_in берём как eff_sr (реальная частота записи)
//   - DECIM = round(Fs_in / 11025)
//   - Fs_out = Fs_in / DECIM
//   - FIR LPF (63 taps, Hamming), затем децимация
//   - тайминг: TIMER_ABSTIME + короткий spin
//   - 1 ioctl на 1 сэмпл
//
// Сборка (Debian/RPi OS):
//   sudo apt-get install -y libgpiod-dev
//   gcc -O2 -Wall -Wextra -std=gnu11 -o ptt_audio ptt_audio.c -lm -lgpiod
//
// Запуск (пример):
//   sudo ./ptt_audio --gpiochip /dev/gpiochip0 --ptt-gpio 27 \
//     --adc /dev/spidev0.0 --dac /dev/spidev0.1 --dac-ch A \
//     --sr 44100 --adc-spi 800000 --dac-spi 2000000 \
//     --chunk 1024 --max-xfers 128 --drop-first --meter \
//     --rx-loop 1 --volume 1.0 --debounce-ms 5 --buf-sec 10
//
// Примечания:
// - PTT активный высокий (3.3V = TX). Можно инвертировать флагом --ptt-active-low
// - В режиме RX PTT можно нажать в любой момент — воспроизведение прерывается и начинается запись.
// - Для Pi1 поднимай DAC SPI частоту (2–8 МГц) если проводка/обвязка позволяет.

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/spi/spidev.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <gpiod.h>

#define CLAMP(v,lo,hi) ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig){ (void)sig; g_stop = 1; }

static inline uint64_t now_ns(void){
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

// ---------- CLI helpers ----------
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
static void usage(const char* a0){
  fprintf(stderr,
    "Usage:\n"
    "  sudo %s --gpiochip /dev/gpiochip0 --ptt-gpio N [--ptt-active-low]\n"
    "    --adc /dev/spidevX.Y --dac /dev/spidevX.Y --dac-ch A|B\n"
    "    --sr 44100 --adc-spi 800000 --dac-spi 2000000\n"
    "    --chunk 1024 --max-xfers 128 [--drop-first] [--meter]\n"
    "    --rx-loop 1 --volume 1.0 --debounce-ms 5 --buf-sec 10\n"
    "\n"
    "Notes:\n"
    "  TX (PTT=1): record from MCP3201 into RAM.\n"
    "  RX (PTT=0): play back from RAM through MCP4822 using \"old wav.c\" logic.\n",
    a0);
}

// ---------- SPI open ----------
static int open_spi_dev(const char* dev, uint32_t* p_hz, uint8_t mode, int* out_fd){
  int fd = open(dev, O_RDWR);
  if(fd<0){ perror("open spidev"); return -1; }
  uint8_t bpw = 8;
  if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) { perror("SPI_IOC_WR_MODE"); close(fd); return -1; }
  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0) { perror("SPI_IOC_WR_BITS_PER_WORD"); close(fd); return -1; }

  uint32_t hz = *p_hz;
  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) { perror("SPI_IOC_WR_MAX_SPEED_HZ"); close(fd); return -1; }
  uint32_t rd=0;
  if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &rd) == 0 && rd) hz = rd;
  *p_hz = hz;
  *out_fd = fd;
  return 0;
}

// ---------- MCP3201 parsing ----------
static inline uint16_t parse_u12_mcp3201(const uint8_t* r2){
  // rx0: ?? 0 B11 B10 B9 B8 B7
  // rx1: B6 B5 B4 B3 B2 B1 B0 x
  uint16_t upper5 = (uint16_t)(r2[0] & 0x1F);
  uint16_t lower7 = (uint16_t)((r2[1] >> 1) & 0x7F);
  return (uint16_t)((upper5 << 7) | lower7);
}

// Read n samples with SPI_IOC_MESSAGE(n) (n transfers), each transfer = 2 bytes.
// cs_change=1 and delay_usecs=tCSH_us to give CS high between conversions.
// This matches what у тебя работало.
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
    trs[i].delay_usecs = tCSH_us;
  }

  int ret = ioctl(fd, SPI_IOC_MESSAGE((int)n), trs);
  return (ret < 0) ? -1 : 0;
}

// ---------- MCP4822 helpers ----------
static inline uint16_t mcp4822_word(int chB, uint16_t u12){
  // [15]=B, [14]=BUF=0, [13]=GA=1 (1x), [12]=SHDN=1, [11:0]=DATA
  return (uint16_t)(((chB?1:0)<<15) | (1<<13) | (1<<12) | (u12 & 0x0FFF));
}
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
static inline uint16_t s16_to_u12_raw12(int16_t s){
  int32_t code_signed12 = ((int32_t)s) >> 4;   // -2048..+2047
  int32_t u = code_signed12 + 2048;           // 0..4095
  if (u<0) u=0; else if (u>4095) u=4095;
  return (uint16_t)u;
}

// ---------- FIR lowpass (как в твоём старом wav.c) ----------
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

// ---------- PTT via libgpiod (polling + debounce) ----------
typedef struct {
  struct gpiod_chip *chip;
  struct gpiod_line *line;
  int active_low;
  int debounce_ms;
  int last_stable;
  uint64_t last_change_ns;
} Ptt;

static int ptt_open(Ptt *p, const char* chip_path, unsigned line_off, int active_low, int debounce_ms){
  memset(p,0,sizeof(*p));
  p->active_low = active_low ? 1 : 0;
  p->debounce_ms = debounce_ms;
  p->chip = gpiod_chip_open(chip_path);
  if(!p->chip){
    perror("gpiod_chip_open");
    return -1;
  }
  p->line = gpiod_chip_get_line(p->chip, (unsigned)line_off);
  if(!p->line){
    perror("gpiod_chip_get_line");
    gpiod_chip_close(p->chip);
    return -1;
  }
  // just input
  if (gpiod_line_request_input(p->line, "ptt_audio") < 0){
    perror("gpiod_line_request_input");
    gpiod_chip_close(p->chip);
    return -1;
  }

  int v = gpiod_line_get_value(p->line);
  if (v < 0) v = 0;
  if (p->active_low) v = !v;
  p->last_stable = v;
  p->last_change_ns = now_ns();
  return 0;
}

static void ptt_close(Ptt *p){
  if(!p) return;
  if(p->chip) gpiod_chip_close(p->chip);
  p->chip = NULL; p->line=NULL;
}

static int ptt_read_raw(Ptt *p){
  int v = gpiod_line_get_value(p->line);
  if (v < 0) v = 0;
  if (p->active_low) v = !v;
  return v ? 1 : 0;
}

// Debounced state: stable for debounce_ms
static int ptt_state(Ptt *p){
  int v = ptt_read_raw(p);
  uint64_t t = now_ns();
  if (v != p->last_stable){
    // start counting stability time
    p->last_change_ns = t;
    p->last_stable = v; // optimistic; we'll enforce stability below by holding last output? -> do better:
  }
  // Better debounce: we need "output" to change only after stable time.
  // We'll keep two vars: stable_out and candidate. To keep code small, do a simple scheme:
  // Store candidate in last_stable and require stability before returning it by holding a static.
  static int out = -1;
  if (out < 0) out = p->last_stable;

  int cand = v;
  if (cand != out){
    // has it been stable long enough?
    uint64_t dt_ns = t - p->last_change_ns;
    if (p->debounce_ms <= 0 || dt_ns >= (uint64_t)p->debounce_ms * 1000000ull){
      out = cand;
    }
  } else {
    // if equal, refresh baseline
    p->last_change_ns = t;
  }
  return out;
}

// ---------- Recording (TX): ADC -> RAM ----------
typedef struct {
  int16_t *pcm;
  size_t cap;
  size_t n;
  double eff_sr;
  uint16_t minv, maxv;
} RecBuf;

static void recbuf_init(RecBuf *r, size_t cap){
  r->pcm = (int16_t*)malloc(cap * sizeof(int16_t));
  r->cap = r->pcm ? cap : 0;
  r->n = 0;
  r->eff_sr = 0.0;
  r->minv = 0xFFFF; r->maxv = 0;
}
static void recbuf_free(RecBuf *r){
  free(r->pcm);
  memset(r,0,sizeof(*r));
}

static void recbuf_clear(RecBuf *r){
  r->n = 0;
  r->eff_sr = 0.0;
  r->minv = 0xFFFF; r->maxv = 0;
}

// simple DC blocker (1st order HPF) like in mic.c
typedef struct { float R, x1, y1; } HPF1;
static void hpf1_init(HPF1* h, float fs, float fc){
  float R = expf(-2.0f*(float)M_PI*fc/fs);
  if(R < 0.0f) R = 0.0f;
  if(R > 0.9999f) R = 0.9999f;
  h->R = R; h->x1 = 0.0f; h->y1 = 0.0f;
}
static float hpf1_run(HPF1* h, float x){
  float y = x - h->x1 + h->R*h->y1;
  h->x1 = x; h->y1 = y; return y;
}

// Record while ptt stays pressed; stop on release or Ctrl+C.
// This is intentionally "mic.c style": max_xfers subcalls per loop.
// Note: we store PCM16 to RAM; eff_sr measured from wall time.
static void do_record_tx(
  Ptt *ptt,
  int adc_fd, uint32_t adc_spi_hz,
  size_t chunk, size_t max_xfers,
  int drop_first, int meter,
  uint32_t sr_target,
  RecBuf *out)
{
  recbuf_clear(out);

  HPF1 hpf; hpf1_init(&hpf, (float)sr_target, 20.0f);
  float gain = 1.0f;

  uint8_t* tx = (uint8_t*)malloc(max_xfers*2);
  uint8_t* rx = (uint8_t*)malloc(max_xfers*2);
  struct spi_ioc_transfer* trs = (struct spi_ioc_transfer*)malloc(max_xfers*sizeof(*trs));
  if(!tx || !rx || !trs){
    fprintf(stderr,"TX: malloc failed\n");
    free(tx); free(rx); free(trs);
    return;
  }

  const uint16_t tCSH_us = 1;

  uint64_t t_start = now_ns();
  uint64_t t0 = t_start;

  fprintf(stderr,"TX: recording... (sr=%u, chunk=%zu, adc_spi=%u, max_xfers=%zu, drop_first=%s)\n",
          sr_target, chunk, adc_spi_hz, max_xfers, drop_first?"ON":"OFF");

  while(!g_stop && ptt_state(ptt)==1){
    size_t remaining = chunk;
    while(remaining && !g_stop && ptt_state(ptt)==1){
      size_t n = remaining;
      if(n > max_xfers) n = max_xfers;

      if(spi_read_samples_mcp3201(adc_fd, adc_spi_hz, tx, rx, trs, n, tCSH_us) != 0){
        perror("TX: SPI_IOC_MESSAGE");
        g_stop = 1;
        break;
      }

      for(size_t i=0;i<n;i++){
        if(out->n >= out->cap) break;

        // drop_first logic: first sample of this ioctl-batch can be "битая" — подменяем на следующую
        uint16_t u12;
        if (drop_first && i==0 && n>=2){
          u12 = parse_u12_mcp3201(rx + 2*1);
        } else {
          u12 = parse_u12_mcp3201(rx + 2*i);
        }

        if(meter){
          if(u12 < out->minv) out->minv = u12;
          if(u12 > out->maxv) out->maxv = u12;
        }

        float s = ((int)u12 - 2048) * (1.0f/2048.0f);
        s = hpf1_run(&hpf, s) * gain;

        if(s > 0.999f) s = 0.999f;
        if(s < -0.999f) s = -0.999f;

        int16_t pcm = (int16_t)lrintf(s * 32767.0f);
        out->pcm[out->n++] = pcm;
      }

      remaining -= n;
      if(out->n >= out->cap) break;
    }

    if(meter){
      uint64_t tn = now_ns();
      if(tn - t0 >= 1000000000ull){
        fprintf(stderr,"TX METER: raw_u12[min=%4u max=%4u], stored=%zu samples\n",
                out->minv==0xFFFF?0:out->minv, out->maxv, out->n);
        out->minv = 0xFFFF; out->maxv = 0;
        t0 = tn;
      }
    }

    if(out->n >= out->cap){
      fprintf(stderr,"TX: buffer full, stopping record.\n");
      break;
    }
  }

  uint64_t t_end = now_ns();
  double wall = (t_end - t_start)/1e9;
  out->eff_sr = (wall > 0.0) ? (out->n / wall) : 0.0;

  fprintf(stderr,"TX done. captured=%zu samples, wall=%.3fs, eff_sr=%.1f Hz\n",
          out->n, wall, out->eff_sr);

  free(tx); free(rx); free(trs);
}

// ---------- Playback (RX): RAM -> DAC using OLD wav.c logic ----------
static void do_play_rx_oldwav(
  Ptt *ptt,
  int dac_fd, uint32_t dac_spi_hz, int dac_chB,
  const int16_t *in_pcm, size_t in_frames,
  double Fs_in,
  float volume,
  int rx_loop)
{
  if(!in_pcm || in_frames < 8 || Fs_in < 1000.0){
    fprintf(stderr,"RX: nothing to play (frames=%zu, Fs_in=%.1f)\n", in_frames, Fs_in);
    // output midscale briefly
    return;
  }

  // compute DECIM like old wav.c
  int DECIM = (int)lround(Fs_in / 11025.0);
  if (DECIM < 1) DECIM = 1;
  double Fs_out = Fs_in / (double)DECIM;

  // timing
  uint64_t period_ns = (uint64_t) llround(1e9 / Fs_out);

  // FIR same as old wav.c (normalized fc=0.22)
  enum { NTAP = 63 };
  float fir[NTAP]; memset(fir,0,sizeof(fir));
  fir_lowpass_hamming(fir, NTAP, 0.22f);

  float rb[NTAP]; memset(rb,0,sizeof(rb));
  int rpos = 0;
  int dec = 0;
  uint32_t rng = 0x12345678u;

  // SPI transfer struct
  uint8_t txb[2];
  struct spi_ioc_transfer tr; memset(&tr,0,sizeof(tr));
  tr.tx_buf        = (uint64_t)(uintptr_t)txb;
  tr.len           = 2;
  tr.speed_hz      = dac_spi_hz;
  tr.bits_per_word = 8;
  tr.cs_change     = 0;
  tr.delay_usecs   = 0;

  // fade lengths (like old: ~5ms)
  const int fade_len = (int)(0.005 * Fs_out + 0.5);
  const int fade_in_len  = fade_len;
  const int fade_out_len = fade_len;

  fprintf(stderr,"RX: play oldwav-logic: in_frames=%zu, Fs_in=%.1f, DECIM=%d, Fs_out=%.2f, vol=%.2f, loop=%s\n",
          in_frames, Fs_in, DECIM, Fs_out, volume, rx_loop?"ON":"OFF");

  do {
    uint64_t t_next = now_ns();
    size_t out_frames = 0;

    // iterate through input frames
    for(size_t i=0; i<in_frames && !g_stop; i++){
      // allow PTT interrupt -> TX
      if (ptt_state(ptt)==1) {
        fprintf(stderr,"RX: PTT pressed -> stop playback\n");
        return;
      }

      float x = (float)in_pcm[i] / 32768.0f;

      if (--rpos < 0) rpos = NTAP-1;
      rb[rpos] = x;

      if (++dec == DECIM){
        dec = 0;

        float acc = 0.0f;
        int idx = rpos;
        for (int k=0; k<NTAP; ++k){
          acc += fir[k] * rb[idx++];
          if (idx == NTAP) idx = 0;
        }

        // fade in/out by output position
        float gain = 1.0f;
        if ((int)out_frames < fade_in_len){
          gain *= (out_frames / (float)MAX(1, fade_in_len));
        }
        // estimate remaining outputs
        int remain_out = (int)((in_frames - i - 1) / (size_t)MAX(1, DECIM));
        if (remain_out < fade_out_len){
          gain *= (remain_out / (float)MAX(1, fade_out_len));
        }
        acc *= gain;

        // timing (absolute)
        t_next += period_ns;
        busy_wait_to(t_next);

        // float -> 12-bit
        int16_t s16 = (int16_t)CLAMP((int)lrintf(acc * 32767.0f), -32768, 32767);
        uint16_t u12 = s16_to_u12_dither(s16, volume, &rng);
        uint16_t w = mcp4822_word(dac_chB, u12);
        txb[0]=(uint8_t)(w>>8); txb[1]=(uint8_t)w;

        if (ioctl(dac_fd, SPI_IOC_MESSAGE(1), &tr) < 0){
          perror("RX: SPI_IOC_MESSAGE(1)");
          return;
        }

        out_frames++;
      }
    }
  } while (rx_loop && !g_stop && ptt_state(ptt)==0);

  fprintf(stderr,"RX: done.\n");
}

int main(int argc, char**argv){
  if(argc < 2){
    usage(argv[0]);
    return 1;
  }

  // defaults
  const char* gpiochip = "/dev/gpiochip0";
  unsigned ptt_gpio = 27;
  int ptt_active_low = 0;
  int debounce_ms = 5;

  const char* adc_dev = "/dev/spidev0.0";
  const char* dac_dev = "/dev/spidev0.1";
  char dac_ch = 'A';

  uint32_t sr_target = 44100;
  uint32_t adc_spi = 800000;
  uint32_t dac_spi = 2000000;
  size_t chunk = 1024;
  size_t max_xfers = 128;
  int drop_first = 0;
  int meter = 0;
  int rx_loop = 1;
  float volume = 1.0f;
  int buf_sec = 10;

  char tmp[128];

  if(get_arg_after(argc, argv, "--gpiochip", tmp, sizeof(tmp))) gpiochip = strdup(tmp);
  if(get_arg_after(argc, argv, "--ptt-gpio", tmp, sizeof(tmp))) ptt_gpio = (unsigned)strtoul(tmp,NULL,10);
  if(has_flag(argc, argv, "--ptt-active-low")) ptt_active_low = 1;

  if(get_arg_after(argc, argv, "--adc", tmp, sizeof(tmp))) adc_dev = strdup(tmp);
  if(get_arg_after(argc, argv, "--dac", tmp, sizeof(tmp))) dac_dev = strdup(tmp);
  if(get_arg_after(argc, argv, "--dac-ch", tmp, sizeof(tmp))) dac_ch = tmp[0];

  if(get_arg_after(argc, argv, "--sr", tmp, sizeof(tmp))) sr_target = (uint32_t)strtoul(tmp,NULL,10);
  if(get_arg_after(argc, argv, "--adc-spi", tmp, sizeof(tmp))) adc_spi = (uint32_t)strtoul(tmp,NULL,10);
  if(get_arg_after(argc, argv, "--dac-spi", tmp, sizeof(tmp))) dac_spi = (uint32_t)strtoul(tmp,NULL,10);

  if(get_arg_after(argc, argv, "--chunk", tmp, sizeof(tmp))) chunk = (size_t)strtoul(tmp,NULL,10);
  if(get_arg_after(argc, argv, "--max-xfers", tmp, sizeof(tmp))) max_xfers = (size_t)strtoul(tmp,NULL,10);

  if(get_arg_after(argc, argv, "--volume", tmp, sizeof(tmp))) volume = strtof(tmp,NULL);
  if(get_arg_after(argc, argv, "--rx-loop", tmp, sizeof(tmp))) rx_loop = (int)strtol(tmp,NULL,10);
  if(get_arg_after(argc, argv, "--debounce-ms", tmp, sizeof(tmp))) debounce_ms = (int)strtol(tmp,NULL,10);
  if(get_arg_after(argc, argv, "--buf-sec", tmp, sizeof(tmp))) buf_sec = (int)strtol(tmp,NULL,10);

  if(has_flag(argc, argv, "--drop-first")) drop_first = 1;
  if(has_flag(argc, argv, "--meter")) meter = 1;
  if(has_flag(argc, argv, "--help")) { usage(argv[0]); return 0; }

  if(chunk < 64) chunk = 64;
  if(chunk > 16384) chunk = 16384;
  if(max_xfers < 1) max_xfers = 1;
  if(max_xfers > 1024) max_xfers = 1024;
  if(volume < 0.01f) volume = 0.01f;
  if(volume > 2.0f) volume = 2.0f;
  if(buf_sec < 1) buf_sec = 1;
  if(buf_sec > 60) buf_sec = 60;

  int dac_chB = (dac_ch=='B' || dac_ch=='b') ? 1 : 0;

  struct sigaction sa; memset(&sa,0,sizeof(sa));
  sa.sa_handler = on_sigint;
  sigaction(SIGINT, &sa, NULL);

  // PTT
  Ptt ptt;
  if(ptt_open(&ptt, gpiochip, ptt_gpio, ptt_active_low, debounce_ms) != 0){
    fprintf(stderr,"PTT open failed.\n");
    return 1;
  }

  // SPI open
  int adc_fd=-1, dac_fd=-1;
  uint32_t adc_spi_drv=adc_spi, dac_spi_drv=dac_spi;

  if(open_spi_dev(adc_dev, &adc_spi_drv, SPI_MODE_0, &adc_fd) != 0){
    ptt_close(&ptt);
    return 1;
  }
  if(open_spi_dev(dac_dev, &dac_spi_drv, SPI_MODE_0, &dac_fd) != 0){
    close(adc_fd);
    ptt_close(&ptt);
    return 1;
  }

  fprintf(stderr,"PTT audio ready.\n");
  fprintf(stderr,"  gpiochip=%s ptt_gpio=%u active_%s debounce=%dms\n",
          gpiochip, ptt_gpio, ptt_active_low?"LOW":"HIGH", debounce_ms);
  fprintf(stderr,"  ADC=%s spi=%u (drv=%u), DAC=%s spi=%u (drv=%u) ch=%c\n",
          adc_dev, adc_spi, adc_spi_drv, dac_dev, dac_spi, dac_spi_drv, dac_ch);
  fprintf(stderr,"  TX: sr_target=%u chunk=%zu max_xfers=%zu drop_first=%s meter=%s\n",
          sr_target, chunk, max_xfers, drop_first?"ON":"OFF", meter?"ON":"OFF");
  fprintf(stderr,"  RX: loop=%s volume=%.2f (old wav.c logic)\n",
          rx_loop?"ON":"OFF", volume);
  fprintf(stderr,"  BUF: %d sec max\n", buf_sec);

  // record buffer capacity: sr_target * buf_sec (worst-case)
  size_t cap = (size_t)sr_target * (size_t)buf_sec;
  if (cap < 4096) cap = 4096;

  RecBuf rb; recbuf_init(&rb, cap);
  if(!rb.pcm){
    fprintf(stderr,"malloc recbuf failed\n");
    close(adc_fd); close(dac_fd); ptt_close(&ptt);
    return 1;
  }

  // main loop
  while(!g_stop){
    int pressed = ptt_state(&ptt);

    if(pressed){
      // TX
      do_record_tx(&ptt, adc_fd, adc_spi_drv, chunk, max_xfers, drop_first, meter, sr_target, &rb);

      // If PTT released quickly, we may have too few samples, still ok.
      // Go back to loop; RX will happen when pressed==0.
    } else {
      // RX
      do_play_rx_oldwav(&ptt, dac_fd, dac_spi_drv, dac_chB, rb.pcm, rb.n, rb.eff_sr, volume, rx_loop);

      // If no data yet, sleep a bit to avoid busy loop
      if (rb.n < 8){
        struct timespec ts = {.tv_sec=0, .tv_nsec=20*1000*1000}; // 20ms
        nanosleep(&ts, NULL);
      }
    }
  }

  fprintf(stderr,"Exiting...\n");
  recbuf_free(&rb);
  close(adc_fd);
  close(dac_fd);
  ptt_close(&ptt);
  return 0;
}

