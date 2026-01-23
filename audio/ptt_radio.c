// ptt_radio_gpio_cs.c — PTT "radio": TX=record from MCP3201, RX=play to MCP4822
// SPI data lines via spidev; CS lines for BOTH ADC and DAC are via GPIO (libgpiod).
// PTT input via libgpiod.
//
// Defaults for NaPi:
//   DAC CS: /dev/gpiochip2 line 4  (GPIO2_A4)
//   ADC CS: /dev/gpiochip2 line 5  (GPIO2_A5)  <-- you can change
//
// Build:
//   gcc -O2 -Wall -Wextra -std=gnu11 -o ptt_radio_gpio_cs ptt_radio_gpio_cs.c -lm -lgpiod
//
// Run example:
//   sudo ./ptt_radio_gpio_cs --gpiochip /dev/gpiochip0 --ptt-gpio 27 \
//     --adc /dev/spidev2.0 --dac /dev/spidev2.0 --dac-ch A \
//     --adc-cs-chip /dev/gpiochip2 --adc-cs-line 5 \
//     --dac-cs-chip /dev/gpiochip2 --dac-cs-line 4 \
//     --sr 12000 --adc-spi 1000000 --dac-spi 1000000 \
//     --chunk 1024 --drop-first --rx-loop 1 --volume 1.0 --meter
//
// Notes:
// - CS via GPIO is slower than HW CS; on NaPi you said it's still fine.
// - We measure real eff_sr while recording and then play at that eff_sr (speed matches).
// - Playback is interruptible by PTT press (immediately switches to TX).

#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sched.h>
#include <gpiod.h>

#define CLAMP(v,lo,hi) ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))

/* ---------------- time helpers ---------------- */
static inline uint64_t now_ns(void){
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec;
}
static inline void ts_from_ns(uint64_t ns, struct timespec* ts){
  ts->tv_sec  = (time_t)(ns/1000000000ull);
  ts->tv_nsec = (long)(ns%1000000000ull);
}
static inline void busy_wait_to(uint64_t t_next){
  uint64_t now = now_ns();
  if (now + 5000ull < t_next){
    struct timespec ts;
    ts_from_ns(t_next - 3000ull, &ts);
    (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
  }
  while ((now = now_ns()) < t_next) { /* spin */ }
}

/* ---------------- CLI tiny helpers ---------------- */
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
static uint32_t u32arg(int argc,char**argv,const char*key,uint32_t defv){
  char tmp[128];
  if(get_arg_after(argc,argv,key,tmp,sizeof(tmp))) return (uint32_t)strtoul(tmp,NULL,10);
  return defv;
}
static int iarg(int argc,char**argv,const char*key,int defv){
  char tmp[128];
  if(get_arg_after(argc,argv,key,tmp,sizeof(tmp))) return (int)strtol(tmp,NULL,10);
  return defv;
}
static float farg(int argc,char**argv,const char*key,float defv){
  char tmp[128];
  if(get_arg_after(argc,argv,key,tmp,sizeof(tmp))) return strtof(tmp,NULL);
  return defv;
}
static const char* sarg_dup(int argc,char**argv,const char*key,const char* defv){
  char tmp[256];
  if(get_arg_after(argc,argv,key,tmp,sizeof(tmp))) return strdup(tmp);
  return strdup(defv);
}


/* ---------------- libgpiod helpers ---------------- */
typedef struct {
  struct gpiod_chip *chip;
  struct gpiod_line *line;
  char chip_path[256];
  unsigned offset;
} GpioLine;

static int gpio_out_open(GpioLine *g, const char* chip_path, unsigned offset, int init_val, const char* consumer){
  memset(g,0,sizeof(*g));
  snprintf(g->chip_path,sizeof(g->chip_path),"%s",chip_path);
  g->offset = offset;

  g->chip = gpiod_chip_open(chip_path);
  if(!g->chip){
    fprintf(stderr,"gpiod_chip_open(%s) failed: %s\n", chip_path, strerror(errno));
    return -1;
  }
  g->line = gpiod_chip_get_line(g->chip, offset);
  if(!g->line){
    fprintf(stderr,"gpiod_chip_get_line(%s,%u) failed: %s\n", chip_path, offset, strerror(errno));
    gpiod_chip_close(g->chip);
    return -1;
  }
  if(gpiod_line_request_output(g->line, consumer, init_val)!=0){
    fprintf(stderr,"gpiod_line_request_output(%s,%u) failed: %s\n", chip_path, offset, strerror(errno));
    gpiod_chip_close(g->chip);
    return -1;
  }
  return 0;
}

static int gpio_in_open(GpioLine *g, const char* chip_path, unsigned offset, const char* consumer){
  memset(g,0,sizeof(*g));
  snprintf(g->chip_path,sizeof(g->chip_path),"%s",chip_path);
  g->offset = offset;

  g->chip = gpiod_chip_open(chip_path);
  if(!g->chip){
    fprintf(stderr,"gpiod_chip_open(%s) failed: %s\n", chip_path, strerror(errno));
    return -1;
  }
  g->line = gpiod_chip_get_line(g->chip, offset);
  if(!g->line){
    fprintf(stderr,"gpiod_chip_get_line(%s,%u) failed: %s\n", chip_path, offset, strerror(errno));
    gpiod_chip_close(g->chip);
    return -1;
  }
  if(gpiod_line_request_input(g->line, consumer)!=0){
    fprintf(stderr,"gpiod_line_request_input(%s,%u) failed: %s\n", chip_path, offset, strerror(errno));
    gpiod_chip_close(g->chip);
    return -1;
  }
  return 0;
}

static void gpio_close(GpioLine *g){
  if(!g) return;
  if(g->line) gpiod_line_release(g->line);
  if(g->chip) gpiod_chip_close(g->chip);
  memset(g,0,sizeof(*g));
}

static inline void gpio_set(GpioLine *g, int v){
  (void)gpiod_line_set_value(g->line, v);
}
static inline int gpio_get(GpioLine *g){
  int v = gpiod_line_get_value(g->line);
  return (v<0)?0:v;
}

/* ---------------- SPI open ---------------- */
static int open_spi_mode0(const char* dev, uint32_t* p_hz, int* out_fd){
  int fd = open(dev, O_RDWR);
  if(fd<0){ perror("open spidev"); return -1; }

  uint8_t mode = SPI_MODE_0;
  uint8_t bpw  = 8;

  if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) { perror("SPI_IOC_WR_MODE"); close(fd); return -1; }
  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0) { perror("SPI_IOC_WR_BITS_PER_WORD"); close(fd); return -1; }

  uint32_t hz = *p_hz;
  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) { perror("SPI_IOC_WR_MAX_SPEED_HZ"); close(fd); return -1; }
  (void)ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &hz);
  *p_hz = hz;

  fprintf(stderr,"SPI opened: %s mode=0, 8b, speed=%u Hz\n", dev, hz);
  *out_fd = fd;
  return 0;
}

/* ---------------- MCP3201 (ADC) ----------------
   Read one sample by toggling CS via GPIO:
   - CS low
   - clock 16 bits (2 bytes) via SPI
   - CS high
   parse:
     rx0: ?? 0 B11 B10 B9 B8 B7
     rx1: B6 B5 B4 B3 B2 B1 B0 x
*/
static inline uint16_t mcp3201_parse_u12(const uint8_t r2[2]){
  uint16_t upper5 = (uint16_t)(r2[0] & 0x1F);
  uint16_t lower7 = (uint16_t)((r2[1] >> 1) & 0x7F);
  return (uint16_t)((upper5 << 7) | lower7);
}

/* ---------------- MCP4822 (DAC) ---------------- */
static inline uint16_t mcp4822_word(int chB, uint16_t u12){
  return (uint16_t)(((chB?1:0)<<15) | (1<<13) | (1<<12) | (u12 & 0x0FFF));
}
static inline uint16_t s16_to_u12_dither(int16_t s, float vol, uint32_t *rng){
  float x = (float)s / 32768.f; x *= vol;
  if (x > 0.999f) x = 0.999f;
  else if (x < -0.999f) x = -0.999f;

  *rng = *rng*1664525u + 1013904223u; float u1=(((*rng>>8)&0xFFFF)/65535.0f);
  *rng = *rng*1664525u + 1013904223u; float u2=(((*rng>>8)&0xFFFF)/65535.0f);
  float d=(u1+u2-1.0f)/4095.0f;
  float y=(x*0.5f + 0.5f) + d;

  int v=(int)lrintf(y*4095.f);
  if (v<0) v=0; else if (v>4095) v=4095;
  return (uint16_t)v;
}

/* ---------------- FIR lowpass (Hamming) ---------------- */
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

/* ---------------- ring buffer (overwrite when full) ---------------- */
typedef struct {
  int16_t *buf;
  size_t cap;        // in samples
  size_t head;       // next write
  size_t size;       // valid samples
} Ring;

static int ring_init(Ring *r, size_t cap){
  memset(r,0,sizeof(*r));
  r->buf = (int16_t*)malloc(cap*sizeof(int16_t));
  if(!r->buf) return -1;
  r->cap = cap;
  r->head = 0;
  r->size = 0;
  return 0;
}
static void ring_free(Ring *r){
  if(r->buf) free(r->buf);
  memset(r,0,sizeof(*r));
}
static void ring_clear(Ring *r){
  r->head = 0;
  r->size = 0;
}
static void ring_push(Ring *r, const int16_t *x, size_t n){
  for(size_t i=0;i<n;i++){
    r->buf[r->head] = x[i];
    r->head = (r->head + 1) % r->cap;
    if(r->size < r->cap) r->size++;
  }
}
// Read i-th oldest sample (0..size-1)
static inline int16_t ring_get_oldest(const Ring *r, size_t i){
  size_t tail = (r->head + r->cap - r->size) % r->cap;
  size_t idx  = (tail + i) % r->cap;
  return r->buf[idx];
}

/* ---------------- PTT debounce (polling) ---------------- */
static int ptt_read_debounced(GpioLine *ptt, int active_high, int debounce_ms){
  int raw = gpio_get(ptt);
  int v = active_high ? raw : !raw;
  if(debounce_ms <= 0) return v;

  // simple debounce: check stable for debounce_ms
  uint64_t t0 = now_ns();
  int last = v;
  for(;;){
    usleep(1000); // 1ms
    raw = gpio_get(ptt);
    v = active_high ? raw : !raw;
    if(v != last){
      last = v;
      t0 = now_ns();
    }else{
      uint64_t dt_ms = (now_ns() - t0)/1000000ull;
      if((int)dt_ms >= debounce_ms) return v;
    }
  }
}

static int open_spi_mode0_lenient(const char* dev, uint32_t* p_hz, int* out_fd){
  int fd = open(dev, O_RDWR);
  if(fd < 0){ perror("open spidev"); return -1; }

  uint8_t mode = SPI_MODE_0;
  uint8_t bpw  = 8;

  if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
    if (errno == EINVAL) {
      fprintf(stderr, "WARN: SPI_IOC_WR_MODE not supported on %s (EINVAL). Continuing.\n", dev);
    } else {
      perror("SPI_IOC_WR_MODE");
      close(fd);
      return -1;
    }
  }

  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0) {
    if (errno == EINVAL) {
      fprintf(stderr, "WARN: SPI_IOC_WR_BITS_PER_WORD not supported on %s (EINVAL). Continuing.\n", dev);
    } else {
      perror("SPI_IOC_WR_BITS_PER_WORD");
      close(fd);
      return -1;
    }
  }

  uint32_t hz = *p_hz;
  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) {
    perror("SPI_IOC_WR_MAX_SPEED_HZ");
    close(fd);
    return -1;
  }
  (void)ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &hz);
  *p_hz = hz;

  fprintf(stderr, "SPI opened: %s (mode=0 requested), 8b, speed=%u Hz\n", dev, hz);
  *out_fd = fd;
  return 0;
}


/* ---------------- ADC record (GPIO CS) ---------------- */
typedef struct {
  uint16_t minv, maxv;
} MeterU12;

static void meter_init(MeterU12 *m){ m->minv=0xFFFF; m->maxv=0; }
static void meter_update(MeterU12 *m, uint16_t v){
  if(v < m->minv) m->minv = v;
  if(v > m->maxv) m->maxv = v;
}

static double record_tx(
  int adc_fd, uint32_t adc_hz,
  GpioLine *adc_cs,
  Ring *ring,
  uint32_t target_sr,
  size_t chunk,
  int drop_first,
  int meter_on,
  int clear_on_ptt,
  int *out_kept,
  int *ptt_release_flag,
  GpioLine *ptt, int ptt_active_high, int debounce_ms
){
  if(clear_on_ptt) ring_clear(ring);

  // SPI transfer setup for 2 bytes
  uint8_t tx[2] = {0,0};
  uint8_t rx[2] = {0,0};
  struct spi_ioc_transfer tr;
  memset(&tr,0,sizeof(tr));
  tr.tx_buf = (uint64_t)(uintptr_t)tx;
  tr.rx_buf = (uint64_t)(uintptr_t)rx;
  tr.len = 2;
  tr.speed_hz = adc_hz;
  tr.bits_per_word = 8;
  tr.cs_change = 0;
  tr.delay_usecs = 0;

  MeterU12 met; meter_init(&met);
  uint64_t t0 = now_ns();
  uint64_t last_meter = t0;

  // drop-first: keep previous valid u12
  uint16_t prev_u12 = 2048;

  size_t total_kept = 0;
  size_t total_raw  = 0;

  int16_t *pcm_chunk = (int16_t*)malloc(chunk * sizeof(int16_t));
  if(!pcm_chunk){ perror("malloc pcm_chunk"); return 0.0; }

  fprintf(stderr,"TX: recording... (target_sr=%u, chunk=%zu, adc_spi=%u, drop_first=%s, cs=%s:%u)\n",
          target_sr, chunk, adc_hz, drop_first?"ON":"OFF", adc_cs->chip_path, adc_cs->offset);

  while(1){
    // Check if PTT still pressed (non-blocking-ish)
    int pressed = ptt_read_debounced(ptt, ptt_active_high, debounce_ms);
    if(!pressed){
      *ptt_release_flag = 1;
      break;
    }

    // fill one chunk (best effort, no pacing in TX)
    for(size_t i=0;i<chunk;i++){
      // CS low
      gpio_set(adc_cs, 0);
      if(ioctl(adc_fd, SPI_IOC_MESSAGE(1), &tr) < 0){
        perror("ADC SPI_IOC_MESSAGE(1)");
        gpio_set(adc_cs, 1);
        *ptt_release_flag = 1;
        goto out;
      }
      // CS high
      gpio_set(adc_cs, 1);

      uint16_t u12 = mcp3201_parse_u12(rx);
      total_raw++;

      if(drop_first){
        // If MSB glitch (classic with some timing), replace suspicious first sample:
        // heuristic: if top bit differs drastically vs previous and value near 0/4095,
        // use prev. (simpler: just ignore first sample after each CS edge isn't possible here)
        // We'll do minimal robust rule: if u12==0 and prev not 0 -> use prev
        // and if u12==4095 and prev not 4095 -> use prev
        if((u12==0 && prev_u12!=0) || (u12==4095 && prev_u12!=4095)){
          u12 = prev_u12;
        }else{
          prev_u12 = u12;
        }
      }

      if(meter_on) meter_update(&met, u12);

      // u12 -> signed PCM16 (center 2048)
      float s = ((int)u12 - 2048) * (1.0f/2048.0f);
      if(s > 0.999f) s = 0.999f;
      if(s < -0.999f) s = -0.999f;
      int16_t pcm = (int16_t)lrintf(s * 32767.0f);

      pcm_chunk[i] = pcm;
    }

    ring_push(ring, pcm_chunk, chunk);
    total_kept += chunk;

    uint64_t now = now_ns();
    if(meter_on && now - last_meter >= 1000000000ull){
      fprintf(stderr,"TX METER: raw_u12[min=%4u max=%4u], stored=%zu samples\n",
              met.minv, met.maxv, ring->size);
      meter_init(&met);
      last_meter = now;
    }

    // allow exit by Ctrl+C? (not implemented here; you can add signal if needed)
  }

out:
  free(pcm_chunk);

  uint64_t t1 = now_ns();
  double wall = (t1 - t0)/1e9;
  double eff_sr = (wall>0)? (total_kept / wall) : 0.0;
  *out_kept = (int)total_kept;

  fprintf(stderr,"TX done. kept=%zu samples, wall=%.3fs, eff_sr=%.1f Hz\n",
          total_kept, wall, eff_sr);

  return eff_sr;
}

/* ---------------- DAC playback (GPIO CS) ----------------
   We play from ring oldest->newest with FIR+DECIM like "old wav.c",
   but Fs_in is measured eff_sr, so speed matches what was recorded.
*/
/* ---------------- DAC playback (GPIO CS) — STREAM, like mic_cs ----------------
   - One DAC update per output sample (=> CS toggles every sample).
   - Output rate = Fs_in (measured eff_sr from TX).
   - No FIR/DECIM here (keeps timing stable, fixes "rare CS" issue).
*/
static void play_rx_stream(
  int dac_fd, uint32_t dac_hz,
  GpioLine *dac_cs,
  const Ring *ring,
  int dac_chB,
  double Fs_in,
  int rx_loop,
  float volume,
  int *ptt_press_flag,
  GpioLine *ptt, int ptt_active_high, int debounce_ms
){
  if(ring->size < 16 || Fs_in < 100.0){
    fprintf(stderr,"RX: nothing to play (ring=%zu, Fs_in=%.1f)\n", ring->size, Fs_in);
    return;
  }

  double Fs_out = Fs_in;                         // IMPORTANT: no decim
  uint64_t period_ns = (uint64_t) llround(1e9 / Fs_out);

  // SPI transfer (2 bytes)
  uint8_t txb[2];
  struct spi_ioc_transfer tr;
  memset(&tr,0,sizeof(tr));
  tr.tx_buf = (uint64_t)(uintptr_t)txb;
  tr.len = 2;
  tr.speed_hz = dac_hz;
  tr.bits_per_word = 8;
  tr.cs_change = 0;
  tr.delay_usecs = 0;

  uint32_t rng = 0x12345678u;

  fprintf(stderr,
    "RX: STREAM ring=%zu, Fs_out≈%.2f Hz, cs=%s:%u, loop=%s, vol=%.2f\n",
    ring->size, Fs_out, dac_cs->chip_path, dac_cs->offset,
    rx_loop?"ON":"OFF", volume
  );

  do {
    uint64_t t0 = now_ns();
    uint64_t t_next = t0;     // absolute schedule
    size_t out_frames = 0;

    for(size_t i=0;i<ring->size;i++){
      // interrupt if PTT pressed
      int pressed = ptt_read_debounced(ptt, ptt_active_high, debounce_ms);
      if(pressed){
        *ptt_press_flag = 1;
        fprintf(stderr,"RX: interrupted by PTT\n");
        return;
      }

      int16_t s16 = ring_get_oldest(ring, i);

      // schedule next sample (ABS time); no "catch-up bursts"
      t_next += period_ns;

      // sleep most of the time, spin the last ~20us for lower jitter
      for(;;){
        uint64_t now = now_ns();
        if(now + 20000ull >= t_next) break; // 20us
        struct timespec ts;
        ts_from_ns(t_next - 15000ull, &ts); // wake slightly early
        if(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == 0) break;
        if(errno != EINTR) break;
      }
      while(now_ns() < t_next) { /* short spin */ }

      // quantize to 12-bit with dither and volume
      uint16_t u12 = s16_to_u12_dither(s16, volume, &rng);
      uint16_t w = mcp4822_word(dac_chB, u12);
      txb[0] = (uint8_t)(w>>8);
      txb[1] = (uint8_t)w;

      // CS via GPIO around SPI frame
      gpio_set(dac_cs, 0);
      if(ioctl(dac_fd, SPI_IOC_MESSAGE(1), &tr) < 0){
        perror("DAC SPI_IOC_MESSAGE(1)");
        gpio_set(dac_cs, 1);
        return;
      }
      gpio_set(dac_cs, 1);

      out_frames++;
    }

    double wall = (now_ns() - t0)/1e9;
    fprintf(stderr,"RX: played %zu frames, wall=%.3fs (expected=%.3fs @ %.2fHz)\n",
            out_frames, wall, out_frames/Fs_out, Fs_out);

  } while(rx_loop);
}


/* ---------------- main ---------------- */
int main(int argc, char**argv){
  if(argc < 2 || has_flag(argc,argv,"--help")){
    fprintf(stderr,
      "Usage:\n"
      "  sudo %s --gpiochip /dev/gpiochip0 --ptt-gpio N [--ptt-active-low]\n"
      "           --adc /dev/spidevX.Y --dac /dev/spidevX.Y --dac-ch A|B\n"
      "           [--adc-cs-chip /dev/gpiochip2 --adc-cs-line 5]\n"
      "           [--dac-cs-chip /dev/gpiochip2 --dac-cs-line 4]\n"
      "           [--sr 12000] [--adc-spi 1000000] [--dac-spi 1000000]\n"
      "           [--chunk 1024] [--buf-sec 10]\n"
      "           [--drop-first] [--no-clear-on-ptt]\n"
      "           [--debounce-ms 20] [--rx-loop 1] [--volume 1.0] [--meter]\n",
      argv[0]
    );
    return 0;
  }

  // PTT
  const char* ptt_chip = sarg_dup(argc,argv,"--gpiochip","/dev/gpiochip0");
  unsigned ptt_gpio = (unsigned)u32arg(argc,argv,"--ptt-gpio",27);
  int ptt_active_high = has_flag(argc,argv,"--ptt-active-low") ? 0 : 1;
  int debounce_ms = iarg(argc,argv,"--debounce-ms",20);

  // SPI devs
  const char* adc_dev = sarg_dup(argc,argv,"--adc","/dev/spidev2.0");
  const char* dac_dev = sarg_dup(argc,argv,"--dac","/dev/spidev2.0");
  const char* dac_chs = sarg_dup(argc,argv,"--dac-ch","A");
  int dac_chB = (dac_chs[0]=='B'||dac_chs[0]=='b');

  uint32_t target_sr = u32arg(argc,argv,"--sr",12000);
  uint32_t adc_hz = u32arg(argc,argv,"--adc-spi",1000000);
  uint32_t dac_hz = u32arg(argc,argv,"--dac-spi",1000000);

  size_t chunk = (size_t)u32arg(argc,argv,"--chunk",1024);
  if(chunk < 64) chunk = 64;
  if(chunk > 16384) chunk = 16384;

  int drop_first = has_flag(argc,argv,"--drop-first");
  int meter_on   = has_flag(argc,argv,"--meter");
  int rx_loop    = (int)u32arg(argc,argv,"--rx-loop",1);
  float volume   = farg(argc,argv,"--volume",1.0f);
  if(volume < 0.01f) volume = 0.01f;
  if(volume > 2.0f)  volume = 2.0f;

  int clear_on_ptt = has_flag(argc,argv,"--no-clear-on-ptt") ? 0 : 1;

  // CS defaults (NaPi)
  const char* adc_cs_chip = sarg_dup(argc,argv,"--adc-cs-chip","/dev/gpiochip2");
  unsigned adc_cs_line    = (unsigned)u32arg(argc,argv,"--adc-cs-line",5);
  const char* dac_cs_chip = sarg_dup(argc,argv,"--dac-cs-chip","/dev/gpiochip2");
  unsigned dac_cs_line    = (unsigned)u32arg(argc,argv,"--dac-cs-line",4);

  // buffer length
  uint32_t buf_sec = u32arg(argc,argv,"--buf-sec",10);
  if(buf_sec < 1) buf_sec = 1;
  if(buf_sec > 120) buf_sec = 120;
  size_t cap = (size_t)buf_sec * (size_t)target_sr;

  // RT best-effort
  /*{
    struct sched_param sp = {.sched_priority=50};
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == 0){
      fprintf(stderr,"RT: SCHED_FIFO enabled, prio=50\n");
    } else {
      fprintf(stderr,"RT: sched_setscheduler(SCHED_FIFO) failed: %s\n", strerror(errno));
    }
  }*/

  // Open PTT GPIO input
  GpioLine ptt;
  if(gpio_in_open(&ptt, ptt_chip, ptt_gpio, "ptt-radio-ptt") != 0) return 1;

  // Open CS lines (idle HIGH)
  GpioLine adc_cs, dac_cs;
  if(gpio_out_open(&adc_cs, adc_cs_chip, adc_cs_line, 1, "ptt-radio-adc-cs") != 0){
    gpio_close(&ptt); return 1;
  }
  if(gpio_out_open(&dac_cs, dac_cs_chip, dac_cs_line, 1, "ptt-radio-dac-cs") != 0){
    gpio_close(&adc_cs); gpio_close(&ptt); return 1;
  }

  // Open SPI
  int adc_fd=-1, dac_fd=-1;
  if(open_spi_mode0_lenient(adc_dev, &adc_hz, &adc_fd) != 0){
    gpio_close(&dac_cs); gpio_close(&adc_cs); gpio_close(&ptt); return 1;
  }
  if(open_spi_mode0_lenient(dac_dev, &dac_hz, &dac_fd) != 0){
    close(adc_fd);
    gpio_close(&dac_cs); gpio_close(&adc_cs); gpio_close(&ptt);
    return 1;
  }

  // Ring
  Ring ring;
  if(ring_init(&ring, cap) != 0){
    fprintf(stderr,"ring_init failed\n");
    close(dac_fd); close(adc_fd);
    gpio_close(&dac_cs); gpio_close(&adc_cs); gpio_close(&ptt);
    return 1;
  }

  fprintf(stderr,
    "PTT radio ready (GPIO-CS for ADC & DAC).\n"
    "  PTT: chip=%s line=%u (%s), debounce=%dms\n"
    "  ADC: %s spi=%u, CS=%s:%u\n"
    "  DAC: %s spi=%u, ch=%c, CS=%s:%u\n"
    "  sr(target)=%u, chunk=%zu, buf_sec=%u (cap=%zu), drop_first=%s, clear_on_ptt=%s\n"
    "  rx_loop=%s, volume=%.2f, meter=%s\n",
    ptt_chip, ptt_gpio, ptt_active_high?"active-high":"active-low", debounce_ms,
    adc_dev, adc_hz, adc_cs_chip, adc_cs_line,
    dac_dev, dac_hz, dac_chB?'B':'A', dac_cs_chip, dac_cs_line,
    target_sr, chunk, buf_sec, cap, drop_first?"ON":"OFF", clear_on_ptt?"ON":"OFF",
    rx_loop?"ON":"OFF", volume, meter_on?"ON":"OFF"
  );

  // state machine
  double last_eff_sr = (double)target_sr;

  while(1){
    // Wait for PTT press
    int pressed = ptt_read_debounced(&ptt, ptt_active_high, debounce_ms);
    if(!pressed){
      usleep(2000); // 2ms poll
      continue;
    }

    // TX: record while pressed
    int kept = 0;
    int released = 0;
    last_eff_sr = record_tx(
      adc_fd, adc_hz,
      &adc_cs,
      &ring,
      target_sr,
      chunk,
      drop_first,
      meter_on,
      clear_on_ptt,
      &kept,
      &released,
      &ptt, ptt_active_high, debounce_ms
    );

    if(kept <= 0){
      fprintf(stderr,"TX: no samples captured\n");
      continue;
    }

    // RX: play while not pressed; interrupt if pressed
    int pressed_again = 0;
    play_rx_stream(
      dac_fd, dac_hz,
      &dac_cs,
      &ring,
      dac_chB,
      last_eff_sr,
      rx_loop,
      volume,
      &pressed_again,
      &ptt, ptt_active_high, debounce_ms
    );

    // loop back; if pressed_again==1, next iteration will go straight into TX
  }

  // never reached
  ring_free(&ring);
  close(dac_fd); close(adc_fd);
  gpio_close(&dac_cs); gpio_close(&adc_cs); gpio_close(&ptt);
  return 0;
}

