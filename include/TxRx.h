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
#include <iostream>
#include <alsa/asoundlib.h>
#include <alsa/pcm.h>

#define DEV_DIR "/dev"
#define BUFFER_SIZE 1024
#define TTY "ttyAMA"
#define PORT 8080
//#define SERVER_IP "192.168.0.105" // IP адрес дом
#define SERVER_IP "10.10.1.211"  // IP адрес работа

void Rx(short * buffer);
void Tx(short * buffer);
void RxEth(short * buffer);
void TxEth(short * buffer);
void audioRxEth(short * buffer);
void audioTxEth(short * buffer);
char * find_ttyUSB_port();



