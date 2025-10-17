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

#define SERVER_IP "10.10.1.62"
//#define SERVER_IP "192.168.31.204"
#define DEV_DIR "/dev"
#define BUFFER_SIZE 2048
#define PERIODS 1024
#define TTY "/dev/ttyUSB0"
#define PORT 5678
#define CHANNELS 2
#define SAMPLERATE 44100
#define RESAMPLE 1
#define CAPTURE_DEV "hw:0,6"
#define PLAYBACK_DEV "hw:0,0"

void Rx(unsigned char * buffer);
void Tx(unsigned char * buffer);
int RxEth(unsigned char * buffer);
void TxEth(unsigned char * buffer);
void audioTxEth_PI(unsigned char * buffer, std::atomic<bool> &running);
void audioRxEth_PI(unsigned char * buffer, std::atomic<bool> &running);
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
