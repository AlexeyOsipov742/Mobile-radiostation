#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/time.h>

int main() {
    const char *port = "/dev/ttyUSB0";
    char buffer[64] = {0};
    int fd = open(port, O_RDWR | O_NOCTTY);  // Не используем O_NONBLOCK

    if (fd == -1) {
        perror("Failed to open port");
        return 1;
    }

    // Настройка порта
    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag &= ~CRTSCTS;         // Отключить аппаратный RTS/CTS
    options.c_cflag |= CREAD | CLOCAL;   // Включить чтение, игнорировать контроль модема

    options.c_lflag = 0;  // raw input
    options.c_iflag = 0;
    options.c_oflag = 0;

    tcsetattr(fd, TCSANOW, &options);

    // Принудительно опускаем RTS вручную
    int status;
    ioctl(fd, TIOCMGET, &status);
    status &= ~TIOCM_RTS;
    ioctl(fd, TIOCMSET, &status);
    usleep(10000); // Небольшая задержка

    printf("Start reading for 1000 ms...\n");

    struct timeval start, now;
 

    while (1) {
        gettimeofday(&start, NULL);
        while(1) {
            gettimeofday(&now, NULL);
            ioctl(fd, TIOCMGET, &status);
            if (status & TIOCM_CTS) {
                long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 +
                                (now.tv_usec - start.tv_usec) / 1000;
                if (elapsed_ms >= 1000) break;
                printf("ms = %ld\n", elapsed_ms);
                // Чтение сигналов RTS и CTS
                ioctl(fd, TIOCMGET, &status);
                printf("RTS: %s | CTS: %s\n",
                    (status & TIOCM_RTS) ? "ON" : "OFF",
                    (status & TIOCM_CTS) ? "ON" : "OFF");

                // Попытка чтения
                int bytes_read = read(fd, buffer, sizeof(buffer));
                if (bytes_read > 0) {
                    printf("Read %d bytes: ", bytes_read);
                    for (int i = 0; i < bytes_read; ++i) {
                        printf("%02x ", (unsigned char)buffer[i]);
                    }
                    printf("\n");
                } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    perror("Read error");
                    break;
                }

                usleep(5000); // Чтобы не грузить CPU
            }
        }
        usleep(10000);
    }

    close(fd);
    return 0;
}
