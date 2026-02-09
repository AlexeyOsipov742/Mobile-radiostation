#pragma once

// =====================
//  Common includes
// =====================
#include <arpa/inet.h>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <thread>
#include <time.h>
#include <unistd.h>

#include <gpiod.h>

// =====================
//  Network / Audio config
// =====================

// IP клиента (NaPi), куда Raspberry отправляет RX-аудио (radio -> NaPi)
//#define SERVER_IP "10.10.1.149"
#define SERVER_IP "192.168.31.20"

#define DEV_DIR "/dev"

// Сетевой пакет аудио: 2048 байт = 1024 семпла S16LE mono
#define BUFFER_SIZE 2048

#define TTY "/dev/ttyUSB0"

// Audio TCP port (и для RX, и для TX)
#define PORT 5678

// Частота дискретизации в проекте (и по сети, и локально)
#ifndef SAMPLERATE
#define SAMPLERATE 8000
#endif

// =====================
//  Raspberry GPIO mapping
// =====================

#ifndef RPI_GPIO_CHIP
#define RPI_GPIO_CHIP "/dev/gpiochip2"
#endif

// COR от радиостанции (в вашем коде активный уровень = LOW)
#ifndef RPI_COR_GPIO
#define RPI_COR_GPIO 15
#endif

// PTT на радиостанцию (в вашем коде активный уровень = HIGH)
#ifndef RPI_PTT_GPIO
#define RPI_PTT_GPIO 16
#endif

// =====================
//  SPI audio (MCP3201 ADC + MCP4822 DAC)
// =====================

// SPI устройство.
// Обычно на Raspberry Pi это /dev/spidev0.0 (SPI0 CE0).
#ifndef RPI_SPI_DEV
#define RPI_SPI_DEV "/dev/spidev2.0"
#endif

// Чип-селекты для АЦП/ЦАП, если вы используете отдельные GPIO линии (НЕ CE0/CE1).
// ВАЖНО: wiringPiSetupGpio() использует BCM-нумерацию!
// Поставьте здесь те номера GPIO, которые реально подключены к CS MCP4822 и CS MCP3201.
#ifndef RPI_DAC_CS_GPIO
#define RPI_DAC_CS_GPIO 4
#endif

#ifndef RPI_ADC_CS_GPIO
#define RPI_ADC_CS_GPIO 5
#endif

// Скорость SPI. 1 MHz обычно стабильно.
#ifndef RPI_SPI_SPEED_HZ
#define RPI_SPI_SPEED_HZ 1000000u
#endif

// Локальная частота для SPI аудио (по умолчанию = SAMPLERATE)
#ifndef RPI_LOCAL_FS
#define RPI_LOCAL_FS SAMPLERATE
#endif

// Усиление входа (ADC) и громкость выхода (DAC). Подстраивается под уровни радиостанции.
#ifndef RPI_RECORD_GAIN
#define RPI_RECORD_GAIN 1.0f
#endif

#ifndef RPI_PLAYBACK_VOL
#define RPI_PLAYBACK_VOL 0.6f
#endif

// Срез DC-blocker (HPF) на входе
#ifndef RPI_HPF_CUTOFF_HZ
#define RPI_HPF_CUTOFF_HZ 20.0f
#endif

// =====================
//  Existing interfaces (UART / control)
// =====================

void Rx(unsigned char *buffer);
void Tx(unsigned char *buffer);
void RxEth(unsigned char *buffer);
void TxEth(unsigned char *buffer);

// Audio directions:
//   - audioTxEth_PI(): radio RX audio (ADC) -> NaPi (TCP client)
//   - audioRxEth_PI(): NaPi TX audio (TCP server) -> radio TX audio (DAC) + PTT
void audioTxEth_PI(unsigned char *buffer);
void audioRxEth_PI(unsigned char *buffer);

char *find_ttyUSB_port();

bool gpio_init();
void gpio_cleanup();

// COR: 0/1 (как в gpiod), но логика COR у тебя "active LOW"
int  gpio_get_cor_level();      // вернёт 0/1
void gpio_set_ptt(int level);   // 0/1

void gpio_set_dac_cs(int level); // 0/1
void gpio_set_adc_cs(int level); // 0/1

