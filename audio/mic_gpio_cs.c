// mic_gpio_cs.c — MCP3201 -> WAV, CS через GPIO (libgpiod)
// SPI: spidev2.0
// CS : gpiochip2 line 5 (GPIO2_A5)
//
// Build:
//   gcc -O2 -Wall -Wextra -std=gnu11 -o mic_gpio_cs mic_gpio_cs.c -lgpiod -lm
//
// Run:
//   sudo ./mic_gpio_cs out.wav 10000 --meter
//
// Stop: Ctrl+C

#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>

/* ---------- globals ---------- */
static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int s){ (void)s; g_stop = 1; }

/* ---------- time ---------- */
static uint64_t now_ns(void){
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec*1000000000ull + ts.tv_nsec;
}

/* ---------- WAV ---------- */
static void put_le32(uint8_t*p,uint32_t v){
  p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24;
}

static void wav_header(FILE*f, uint32_t sr){
  uint8_t h[44]={0};
  memcpy(h,"RIFF",4); memcpy(h+8,"WAVEfmt ",8);
  put_le32(h+16,16); h[20]=1; h[22]=1;
  put_le32(h+24,sr); put_le32(h+28,sr*2);
  h[32]=2; h[34]=16;
  memcpy(h+36,"data",4);
  fwrite(h,1,44,f);
}

static void wav_finalize(FILE*f){
  long sz = ftell(f);
  uint32_t data = sz - 44;
  uint32_t riff = data + 36;
  fseek(f,4,SEEK_SET); put_le32((uint8_t*)&riff,riff); fwrite(&riff,4,1,f);
  fseek(f,40,SEEK_SET); put_le32((uint8_t*)&data,data); fwrite(&data,4,1,f);
}

/* ---------- MCP3201 ---------- */
static inline uint16_t mcp3201_parse(const uint8_t r[2]){
  return ((r[0] & 0x1F) << 7) | ((r[1] >> 1) & 0x7F);
}

/* ---------- main ---------- */
int main(int argc,char**argv){
  if(argc < 3){
    fprintf(stderr,"Usage: %s out.wav samplerate [--meter]\n",argv[0]);
    return 1;
  }

  const char* outwav = argv[1];
  uint32_t sr = strtoul(argv[2],NULL,10);
  int meter = (argc>=4 && strcmp(argv[3],"--meter")==0);

  /* SPI */
  int sfd = open("/dev/spidev2.0",O_RDWR);
  if(sfd<0){ perror("open spidev"); return 1; }

  uint8_t mode = SPI_MODE_0, bpw=8;
  uint32_t hz = 1000000;
  ioctl(sfd,SPI_IOC_WR_MODE,&mode);
  ioctl(sfd,SPI_IOC_WR_BITS_PER_WORD,&bpw);
  ioctl(sfd,SPI_IOC_WR_MAX_SPEED_HZ,&hz);

  /* GPIO CS */
  struct gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip2");
  if(!chip){ perror("gpiod_chip_open"); return 1; }

  struct gpiod_line* cs = gpiod_chip_get_line(chip,5);
  if(!cs){ perror("get_line"); return 1; }

  if(gpiod_line_request_output(cs,"mcp3201-cs",1)!=0){
    perror("request_output"); return 1;
  }

  FILE* f = fopen(outwav,"wb");
  if(!f){ perror("fopen"); return 1; }
  wav_header(f,sr);

  signal(SIGINT,on_sigint);

  uint8_t tx[2]={0,0}, rx[2]={0,0};
  struct spi_ioc_transfer tr = {
    .tx_buf=(uintptr_t)tx,
    .rx_buf=(uintptr_t)rx,
    .len=2,
    .speed_hz=hz,
    .bits_per_word=8
  };

  uint64_t period = (uint64_t)(1e9/sr);
  uint64_t next = now_ns();

  uint16_t minv=4095,maxv=0;
  uint64_t last = now_ns();

  fprintf(stderr,"Recording MCP3201 via GPIO-CS @ %u Hz\n",sr);

  while(!g_stop){
    gpiod_line_set_value(cs,0);
    ioctl(sfd,SPI_IOC_MESSAGE(1),&tr);
    gpiod_line_set_value(cs,1);

    uint16_t u12 = mcp3201_parse(rx);
    if(u12<minv) minv=u12;
    if(u12>maxv) maxv=u12;

    float s = ((int)u12 - 2048) / 2048.0f;
    if(s>1) s=1; if(s<-1) s=-1;
    int16_t pcm = (int16_t)lrintf(s*32767.0f);
    fwrite(&pcm,2,1,f);

    next += period;
    while(now_ns()<next);

    if(meter && now_ns()-last > 1000000000ull){
      fprintf(stderr,"METER raw_u12[min=%4u max=%4u]\n",minv,maxv);
      minv=4095; maxv=0; last=now_ns();
    }
  }

  wav_finalize(f);
  fclose(f);
  close(sfd);
  gpiod_line_release(cs);
  gpiod_chip_close(chip);

  fprintf(stderr,"Done.\n");
  return 0;
}

