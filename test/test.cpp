#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>

int main() {
    struct termios options;
    char buffer[64] = {0};
    char *port = "/dev/ttyUSB0";
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd == -1) {
        perror("Failed to open port");
        return 1;
    }

    // Настройка порта
    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag |= CS8;
    options.c_cflag |= CRTSCTS;  // аппаратный контроль потока

    tcsetattr(fd, TCSANOW, &options);

    // Опускаем RTS (чтобы станция отвечала)
    int status;
    ioctl(fd, TIOCMGET, &status);
    status &= ~TIOCM_RTS;
    ioctl(fd, TIOCMSET, &status);
    usleep(10000);
    
    printf("Начинаем чтение с порта циклами по 100 мс...\n");

    while (1) {
        struct timeval start_time, now;
        gettimeofday(&start_time, NULL);  // Засекаем старт времени
        ioctl(fd, TIOCMGET, &status);
        if (status & TIOCM_CTS) {
            while (1) {
                ioctl(fd, TIOCMGET, &status);
                printf("RTS: %s | CTS: %s\n",
                    (status & TIOCM_RTS) ? "ON" : "OFF",
                    (status & TIOCM_CTS) ? "ON" : "OFF");
                gettimeofday(&now, NULL);  // Засекаем текущее
                long elapsed_ms = (now.tv_sec - start_time.tv_sec) * 1000 +
                                (now.tv_usec - start_time.tv_usec) / 1000;

                if (elapsed_ms >= 1000) break;  // Если прошло 100 мс — выходим

                printf("ms = %ld\n", elapsed_ms);  // Печать прошедшего времени

                int bytes_read = read(fd, buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';
                    printf("Ответ станции (%d байт): ", bytes_read);
                    for (int i = 0; i < bytes_read; i++) {
                        printf("%02x ", (unsigned char)buffer[i]);
                    }
                    printf("\n");
                } else if (bytes_read == 0) {
                    printf("read: no data (0 bytes)\n");
                } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    perror("Read error");
                    break;
                }

                usleep(1000);  // чтобы не грузить CPU
            }
        }

        //usleep(200000);  // пауза 200 мс между циклами (по желанию)
    }

    close(fd);
    return 0;
}
