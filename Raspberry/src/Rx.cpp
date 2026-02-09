#include "TxRx.h"


void Rx(unsigned char * buffer) {
    char *port = TTY;
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd == -1) {
        perror("Failed to open port");
    }

    char local_buffer[64] = {0};
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
    /*ioctl(fd, TIOCMGET, &status);
    status |= TIOCM_RTS;
    ioctl(fd, TIOCMSET, &status);
    usleep(10000); // Небольшая задержка

    int bytes_written = write(fd, message, sizeof(message));

    if (bytes_written < 0) {
        perror("Error writing to port - ");
        close(fd);
    }
    usleep(10000);
    */

    ioctl(fd, TIOCMGET, &status);
    status &= ~TIOCM_RTS;
    ioctl(fd, TIOCMSET, &status);
    usleep(10000); // Небольшая задержка

    printf("Start reading for 300 ms...\n");

    //int offset = 0;

    struct timeval start, now;
    int buffer_counter = 0;
    gettimeofday(&start, NULL);
    while(1) {
        gettimeofday(&now, NULL);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 +
                            (now.tv_usec - start.tv_usec) / 1000;
        if (elapsed_ms >= 300) break;
        printf("ms = %ld\n", elapsed_ms);

        ioctl(fd, TIOCMGET, &status);
        if (status & TIOCM_CTS) {
            // Чтение сигналов RTS и CTS
            /*ioctl(fd, TIOCMGET, &status);
            printf("RTS: %s | CTS: %s\n",
                (status & TIOCM_RTS) ? "ON" : "OFF",
                (status & TIOCM_CTS) ? "ON" : "OFF");
            */
            // Попытка чтения
            int bytes_read = read(fd, local_buffer, sizeof(local_buffer));
            if (bytes_read > 0) {
                printf("Read %d bytes: ", bytes_read);
                for (int i = 0; i < bytes_read; ++i) {
                    printf("%02x ", (unsigned char)local_buffer[i]);
                }
                printf("\n");
                printf("Read %d bytes: ", bytes_read);
                for (int i = 0; i < bytes_read; ++i) {
                    printf("%c ", local_buffer[i]);
                }
                printf("\n");
                //offset += bytes_read;
                for (int i = 0; i < bytes_read; i++) {
                    buffer[i + buffer_counter] = local_buffer[i];
                }
                buffer_counter += bytes_read;
                memset(local_buffer, 0, sizeof(local_buffer));
                
            } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("Read error");
                printf("Error reading\n");
            }
            
            usleep(5000); // Чтобы не грузить CPU
        }
    }
    printf("END OF RX\n");

    close(fd);
}
