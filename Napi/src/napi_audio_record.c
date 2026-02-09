// adc_mcp3202_rec.c — Захват аудио с MCP3202 (SPI) в WAV (S16_LE) «как arecord -r44100».
// Сборка: gcc -O2 -Wall -pthread -o mic adc_mcp3202_rec.c -lm
#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <stddef.h>   // <-- ДОБАВЛЕНО для offsetof
#include <time.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int){ g_stop = 1; }

#pragma pack(push,1)
typedef struct {
  uint32_t riff_id;     // "RIFF"
  uint32_t riff_sz;     // 4 + (8+Subchunk1) + (8+Subchunk2)
  uint32_t wave_id;     // "WAVE"
  uint32_t fmt_id;      // "fmt "
  uint32_t fmt_sz;      // 16
  uint16_t audio_fmt;   // 1=PCM
  uint16_t num_ch;      // 1
  uint32_t sample_rate; // e.g. 44100
  uint32_t byte_rate;   // sample_rate*num_ch*bytes_per_sample
  uint16_t block_align; // num_ch*bytes_per_sample
  uint16_t bits_per_sample; // 16
  uint32_t data_id;     // "data"
  uint32_t data_sz;     // num_samples*num_ch*bytes_per_sample
} WavHeader;
#pragma pack(pop)

static int write_wav_header(FILE* f, uint32_t sr){
  WavHeader h;
  h.riff_id = 0x46464952; // "RIFF"
  h.riff_sz = 0;
  h.wave_id = 0x45564157; // "WAVE"
  h.fmt_id  = 0x20746d66; // "fmt "
  h.fmt_sz  = 16;
  h.audio_fmt = 1;
  h.num_ch = 1;
  h.sample_rate = sr;
  h.bits_per_sample = 16;
  h.block_align = h.num_ch * (h.bits_per_sample/8);
  h.byte_rate = h.sample_rate * h.block_align;
  h.data_id = 0x61746164; // "data"
  h.data_sz = 0;
  return (fwrite(&h, sizeof(h), 1, f) == 1) ? 0 : -1;
}

static int finalize_wav(FILE* f){
  long end = ftell(f);
  if (end < 0) return -1;
  uint32_t data_sz = (uint32_t)(end - sizeof(WavHeader));
  uint32_t riff_sz = data_sz + sizeof(WavHeader) - 8;

  if (fseek(f, 4, SEEK_SET)!=0) return -1;
  if (fwrite(&riff_sz, 4, 1, f)!=1) return -1;
  if (fseek(f, (long)offsetof(WavHeader, data_sz), SEEK_SET)!=0) return -1;  // <-- теперь работает
  if (fwrite(&data_sz, 4, 1, f)!=1) return -1;
  fflush(f);
  return 0;
}

// --- SPI / MCP3202 вспомогательные (как у вас было) ---
static inline void build_cmd_triplet(uint8_t *p, int ch){
  p[0] = 0x01;
  p[1] = (uint8_t)(0xA0 | ((ch?1:0) << 6)); // CH0->0xA0, CH1->0xE0
  p[2] = 0x00;
}
static inline uint16_t parse_u12(const uint8_t *p){
  return (uint16_t)(((p[1] & 0x0F) << 8) | p[2]);
}

typedef struct { float a; float x1, y1; } HPF1;
static inline void hpf1_init(HPF1 *f, float sr, float fc){
  float alpha = expf(-2.0f * (float)M_PI * fc / sr);
  f->a = alpha; f->x1 = 0.0f; f->y1 = 0.0f;
}
static inline float hpf1_run(HPF1 *f, float x){
  float y = (1.0f - f->a) * (x - f->x1) + f->a * f->y1;
  f->x1 = x; f->y1 = y; return y;
}

// --- Очередь писателя (без изменений по сути) ---
typedef struct {
  FILE *wav;
  int16_t *pcm_q[4];
  size_t   count_q[4];
  int head, tail;
  pthread_mutex_t m;
  pthread_cond_t  cv;
  int stop;
} WriterQ;

static void wq_init(WriterQ *q){
  memset(q,0,sizeof(*q));
  pthread_mutex_init(&q->m,NULL);
  pthread_cond_init(&q->cv,NULL);
}
static void wq_push(WriterQ *q, int16_t *buf, size_t n){
  pthread_mutex_lock(&q->m);
  int next = (q->head+1)&3;
  while (next == q->tail){
    pthread_cond_wait(&q->cv, &q->m);
  }
  q->pcm_q[q->head] = buf;
  q->count_q[q->head] = n;
  q->head = next;
  pthread_cond_signal(&q->cv);
  pthread_mutex_unlock(&q->m);
}
static int wq_pop(WriterQ *q, int16_t **buf, size_t *n){
  pthread_mutex_lock(&q->m);
  while (q->tail == q->head && !q->stop){
    pthread_cond_wait(&q->cv, &q->m);
  }
  if (q->tail == q->head && q->stop){ pthread_mutex_unlock(&q->m); return 0; }
  *buf = q->pcm_q[q->tail];
  *n   = q->count_q[q->tail];
  q->tail = (q->tail+1)&3;
  pthread_cond_signal(&q->cv);
  pthread_mutex_unlock(&q->m);
  return 1;
}
static void wq_close(WriterQ *q){
  pthread_mutex_lock(&q->m); q->stop = 1; pthread_cond_broadcast(&q->cv); pthread_mutex_unlock(&q->m);
}
typedef struct { WriterQ *q; } WriterArgs;
static void* writer_thread(void*arg){
  WriterArgs *wa = (WriterArgs*)arg;
  int16_t *buf; size_t n;
  while (wq_pop(wa->q, &buf, &n)){
    if (n){ fwrite(buf, sizeof(int16_t), n, wa->q->wav); }
    free(buf);
  }
  return NULL;
}

static int open_spi(const char*dev, uint32_t *hz_inout, int *fd_out){
  int fd = open(dev, O_RDWR);
  if (fd < 0){ perror("open spidev"); return -1; }
  uint8_t mode = SPI_MODE_0, bpw = 8;
  if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1){ perror("SPI_IOC_WR_MODE"); close(fd); return -1; }
  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) == -1){ perror("SPI_IOC_WR_BITS_PER_WORD"); close(fd); return -1; }
  uint32_t hz = *hz_inout;
  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) == -1){ perror("SPI_IOC_WR_MAX_SPEED_HZ"); close(fd); return -1; }
  uint32_t rd=0; if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &rd) == -1) rd=hz;
  if (rd && rd!=*hz_inout) fprintf(stderr,"WARN: requested %u Hz, driver reports %u Hz\n", *hz_inout, rd);
  if (rd) *hz_inout = rd;
  *fd_out = fd;
  return 0;
}

int main(int argc, char**argv){
  if (argc < 4){
    fprintf(stderr,"Usage:\n  %s <spidev> <A|B> <out.wav> [sample_rate=44100] [gain=1.0] [block_samples=4096]\n", argv[0]);
    return 1;
  }
  const char* spidev = argv[1];
  int ch = (argv[2][0]=='B'||argv[2][0]=='b') ? 1 : 0;
  const char* outwav = argv[3];
  uint32_t sample_rate = (argc>=5)? (uint32_t)strtoul(argv[4],NULL,10) : 44100u;
  float gain = (argc>=6)? strtof(argv[5],NULL) : 1.0f;
  if (gain < 0.1f) gain = 0.1f;
  if (gain > 16.0f) gain = 16.0f;   // <-- разнесено на две строки

  size_t block_samp = (argc>=7)? (size_t)strtoul(argv[6],NULL,10) : 4096u;
  if (block_samp < 256) block_samp = 256;

  FILE* f = fopen(outwav, "wb");
  if (!f){ perror("fopen wav"); return 1; }
  if (write_wav_header(f, sample_rate)!=0){ fprintf(stderr,"write_wav_header failed\n"); fclose(f); return 1; }

  uint32_t spi_hz = sample_rate * 24u;
  int sfd; if (open_spi(spidev, &spi_hz, &sfd)!=0){ fclose(f); return 1; }
  fprintf(stderr,"SPI mode 0, 8b, speed=%u Hz (target %.0f Hz)\n", spi_hz, (double)sample_rate*24.0);

  const size_t BYTES_PER_SMP = 3;
  size_t txrx_len = BYTES_PER_SMP * block_samp;
  uint8_t* tx = (uint8_t*)malloc(txrx_len);
  if (!tx){ perror("malloc tx"); close(sfd); fclose(f); return 1; }
  for (size_t i=0; i<block_samp; ++i) build_cmd_triplet(tx + 3*i, ch);

  WriterQ q; wq_init(&q); q.wav = f;
  WriterArgs wa = { .q = &q };
  pthread_t th; pthread_create(&th, NULL, writer_thread, &wa);

  HPF1 hpf; hpf1_init(&hpf, (float)sample_rate, 20.0f);

  struct sigaction sa; memset(&sa,0,sizeof(sa)); sa.sa_handler=on_sigint; sigaction(SIGINT,&sa,NULL);

  struct spi_ioc_transfer tr; memset(&tr,0,sizeof(tr));
  tr.tx_buf = (uint64_t)(uintptr_t)tx;
  tr.len = (uint32_t)txrx_len;
  tr.speed_hz = spi_hz;
  tr.bits_per_word = 8;
  tr.cs_change = 0;
  tr.delay_usecs = 0;

  size_t total_samples = 0;
  fprintf(stderr, "Recording… CH%s, Fs=%u Hz, block=%zu samples. Ctrl+C to stop.\n",
          ch?"1(B)":"0(A)", sample_rate, block_samp);

  while (!g_stop){
    uint8_t* rx = (uint8_t*)malloc(txrx_len);
    if (!rx){ perror("malloc rx"); break; }
    tr.rx_buf = (uint64_t)(uintptr_t)rx;

    if (ioctl(sfd, SPI_IOC_MESSAGE(1), &tr) < 0){
      perror("SPI_IOC_MESSAGE"); free(rx); break;
    }

    int16_t* pcm = (int16_t*)malloc(block_samp * sizeof(int16_t));
    if (!pcm){ perror("malloc pcm"); free(rx); break; }

    for (size_t i=0; i<block_samp; ++i){
      uint16_t u12 = parse_u12(rx + 3*i);
      float s = ((int)u12 - 2048) * (1.0f/2048.0f);
      s = hpf1_run(&hpf, s) * gain;
      if (s > 0.999f) s = 0.999f;           // <-- разнесено
      if (s < -0.999f) s = -0.999f;         // <-- разнесено
      pcm[i] = (int16_t)lrintf(s * 32767.0f);
    }
    free(rx);

    total_samples += block_samp;
    wq_push(&q, pcm, block_samp);
  }

  wq_close(&q);
  pthread_join(th, NULL);

  if (finalize_wav(f)!=0) fprintf(stderr,"WARN: finalize_wav failed\n");
  fclose(f);
  close(sfd);
  free(tx);

  fprintf(stderr,"Done. Wrote %zu samples (%.2f s) to %s\n",
          total_samples, total_samples/(double)sample_rate, outwav);
  return 0;
}
