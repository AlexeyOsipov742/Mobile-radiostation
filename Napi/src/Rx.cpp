// Rx.cpp — чтение с UART (SB9600) в агрегирующий буфер на 300 мс
// Сигнатура оставлена как у вас: void Rx(unsigned char* buffer)
//
// Зависимости: TTY и BUFFER_SIZE могут приходить из TxRx.h.
// Если их нет — используем дефолты ниже.

#include "TxRx.h"

#include <cstdio>
#include <cstring>
#include <cerrno>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#ifndef TTY
#define TTY "/dev/ttyUSB0"
#endif

#ifndef BUFFER_SIZE
#define BUFFER_SIZE 2048
#endif

static void dump_hex_chunk(const unsigned char* p, int n) {
    for (int i = 0; i < n; ++i) {
        std::printf("%02x ", (unsigned)p[i]);
    }
    std::printf("\n");
}

void Rx(unsigned char* buffer) {
    // Открываем порт (неблокирующий режим нам удобнее для «окна» на 300 мс)
    int fd = ::open(TTY, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("Failed to open port");
        return;
    }
    std::printf("Found ttyUSB port: %s\n", TTY);

    // Настройка termios: 9600 8N1, без аппаратного флоуконтроля, RAW
    struct termios tio{};
    if (tcgetattr(fd, &tio) < 0) perror("tcgetattr");

    cfmakeraw(&tio);
    cfsetispeed(&tio, B9600);
    cfsetospeed(&tio, B9600);

    tio.c_cflag |= (CLOCAL | CREAD | CS8);
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

    // Без каноники, без преобразований
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0;

    if (tcsetattr(fd, TCSANOW, &tio) < 0) perror("tcsetattr");

    // Принудительно опустим RTS (иногда полезно на переходниках)
    int status = 0;
    if (ioctl(fd, TIOCMGET, &status) == 0) {
        status &= ~TIOCM_RTS;
        ioctl(fd, TIOCMSET, &status);
    }

    // Подготовим общий буфер
    unsigned char local[64];
    std::memset(local, 0, sizeof(local));
    std::memset(buffer, 0, BUFFER_SIZE);
    int buffer_counter = 0;

    std::printf("Start reading for 300 ms...\n");

    // Окно 300 мс
    timeval start{}, now{};
    gettimeofday(&start, nullptr);

    while (true) {
        gettimeofday(&now, nullptr);
        long elapsed_ms =
            (now.tv_sec - start.tv_sec) * 1000L +
            (now.tv_usec - start.tv_usec) / 1000L;
        if (elapsed_ms >= 300) break;

        // Читаем неблокирующе
        ssize_t n = ::read(fd, local, sizeof(local));
        if (n > 0) {
            // Дамп прочитанного куска
            std::printf("Read %zd bytes: ", n);
            dump_hex_chunk(local, (int)n);

            std::printf("Read %zd bytes: ", n);
            for (ssize_t i = 0; i < n; ++i) {
                unsigned char c = local[i];
                std::printf("%c ", (c >= 32 && c <= 126) ? c : ' ');
            }
            std::printf("\n");

            // Копируем в общий буфер с защитой от переполнения
            int room = BUFFER_SIZE - buffer_counter;
            if (room <= 0) break; // буфер заполнен
            if (n > room) n = room;

            std::memcpy(buffer + buffer_counter, local, (size_t)n);
            buffer_counter += (int)n;

            // Обнулим локальный, чтобы читаемость логов была лучше
            std::memset(local, 0, sizeof(local));
        } else if (n == 0) {
            // нет данных — короткая пауза
            usleep(5000);
        } else {
            // n < 0
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                usleep(5000);
            } else {
                perror("Read error");
                break;
            }
        }
    }

    // Отладочный сводный дамп: ровно полученные байты
    std::printf("\n[AGG HEX %d bytes]\n", buffer_counter);
    for (int i = 0; i < buffer_counter; ++i) {
        std::printf("%02x ", buffer[i]);
        if ((i % 32) == 31) std::printf("\n");
    }
    std::printf("\n[AGG ASCII]\n");
    for (int i = 0; i < buffer_counter; ++i) {
        unsigned char c = buffer[i];
        std::putchar((c >= 32 && c <= 126) ? (char)c : '.'); // точки вместо нулей
    }
    std::printf("\nEND OF RX\n");

    // На случай, если вызывающая сторона по-старому шлёт ровно 2048,
    // оставшаяся часть уже обнулена memset'ом выше.

    ::close(fd);
}

