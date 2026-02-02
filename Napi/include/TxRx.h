#pragma once

#include <cmath>
#include <csignal>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <atomic>
#include <csignal>
#include <thread>
#include <mutex>

//#define SERVER_IP "10.10.1.149"
#define SERVER_IP "192.168.31.20"
#define DEV_DIR "/dev"
#define BUFFER_SIZE 2048
#define PERIODS 1024
#define TTY "/dev/ttyUSB0"
#define PORT 5678
#define CHANNELS 1
#define SAMPLERATE 8000
#define RESAMPLE 1
#define CAPTURE_DEV "hw:0,6"
#define PLAYBACK_DEV "hw:0,0"

// ===== NaPi audio (SPI + GPIO-CS) =====
// На NaPi доступна только одна SPI-шина, поэтому CS для ЦАП/АЦП управляем через GPIO.
// Эти значения совпадают с тестовыми программами из папки audio/.
// При необходимости можно поменять под свою разводку.
#ifndef NAPI_SPI_DEV
#define NAPI_SPI_DEV "/dev/spidev2.0"
#endif

#ifndef NAPI_DAC_CS_CHIP
#define NAPI_DAC_CS_CHIP "/dev/gpiochip2"
#endif
#ifndef NAPI_DAC_CS_LINE
#define NAPI_DAC_CS_LINE 4
#endif

#ifndef NAPI_ADC_CS_CHIP
#define NAPI_ADC_CS_CHIP "/dev/gpiochip2"
#endif
#ifndef NAPI_ADC_CS_LINE
#define NAPI_ADC_CS_LINE 5
#endif

// Локальная частота для ЦАП/АЦП (в тестах хорошо работает 11025/12000).
// По сети оставляем 44100 (SAMPLERATE), а на TX делаем upsample x4.
#ifndef NAPI_LOCAL_FS
#define NAPI_LOCAL_FS 8000
#endif

// Сколько каналов приходит по сети в режиме RX (от RaspberryPi).
// В текущем серверном коде захват идет как stereo (2ch).
#ifndef NAPI_RX_NET_CHANNELS
#define NAPI_RX_NET_CHANNELS 1
#endif

void Rx(unsigned char * buffer);
void Tx(unsigned char * buffer);
int RxEth(unsigned char * buffer);
void TxEth(unsigned char * buffer);
void audioTxEth_PI(unsigned char * buffer, std::atomic<bool> &running);
void audioRxEth_PI(unsigned char * buffer, std::atomic<bool> &running);
void audioTxEth_client(unsigned char * buffer, std::atomic<bool> &running);
void audioRxEth_client(unsigned char * buffer, std::atomic<bool> &running);
char * find_ttyUSB_port();
void audio(std::atomic<bool> &running);
void command(std::atomic<bool> &running);

extern std::mutex uart_mutex;
extern std::atomic<bool> audio_running;
extern std::atomic<bool> cmd_running;

void signal_handler(int signal);
bool gpio_init();
void gpio_cleanup();
int gpio_get_ptt_level();
void gpio_set_activity_led(bool on);
