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

int Rx();
int Tx();
char *find_ttyUSB_port();
int RxEth();
int TxEth();



