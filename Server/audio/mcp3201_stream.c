// mcp3201_stream.c — непрерывное чтение MCP3201 через Linux spidev и вывод кода 0..4095.
//
// Сборка:
//   gcc -O2 -Wall -std=gnu11 -o mcp3201_stream mcp3201_stream.c
//
// Запуск (примеры):
//   ./mcp3201_stream /dev/spidev2.0
//   ./mcp3201_stream /dev/spidev2.0 1000        # delay_us между выборками
//   ./mcp3201_stream /dev/spidev2.0 1000 1000000 0   # delay_us, speed_hz, spi_mode(0 или 3)
//
// Остановить: Ctrl+C.

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

static void die(const char *msg) {
    fprintf(stderr, "%s: %s\n", msg, strerror(errno));
    exit(1);
}

// Чтение MCP3201 12-bit MSB-first.
// Делаем 16 тактов (2 байта), как в примере даташита с 8-битными сегментами.
// В первом байте:  ?? 0 B11 B10 B9 B8 B7
// Во втором байте: B6 B5 B4 B3 B2 B1 B0 B1(повтор, т.к. лишний такт начинает LSB-first)
// Поэтому код собираем так:
//   upper5 = (rx0 & 0x1F) = B11..B7
//   lower7 = (rx1 >> 1)  = B6..B0
//   code = (upper5 << 7) | lower7
static int mcp3201_read12(int fd, uint32_t speed_hz, uint16_t *out)
{
    uint8_t tx[2] = {0x00, 0x00};
    uint8_t rx[2] = {0x00, 0x00};

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = sizeof(tx),
        .delay_usecs = 0,
        .speed_hz = speed_hz,
        .bits_per_word = 8,
        .cs_change = 0,
    };

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) return -1;

    uint16_t upper5 = (uint16_t)(rx[0] & 0x1F);        // B11..B7
    uint16_t lower7 = (uint16_t)((rx[1] >> 1) & 0x7F); // B6..B0
    *out = (uint16_t)((upper5 << 7) | lower7);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr,
                "Usage: %s /dev/spidevX.Y [delay_us] [speed_hz] [spi_mode]\n"
                "  spi_mode: 0 (MODE0) or 3 (MODE3). MCP3201 supports 0,0 and 1,1.\n"
                "Example: %s /dev/spidev2.0 1000 1000000 0\n",
                argv[0], argv[0]);
        return 2;
    }

    const char *dev = argv[1];

    int delay_us = (argc >= 3) ? atoi(argv[2]) : 0;
    if (delay_us < 0) delay_us = 0;

    uint32_t speed_hz = (argc >= 4) ? (uint32_t)strtoul(argv[3], NULL, 10) : 1000000; // 1 MHz
    if (speed_hz == 0) speed_hz = 1000000;

    int mode_arg = (argc >= 5) ? atoi(argv[4]) : 0;
    uint8_t mode = (mode_arg == 3) ? SPI_MODE_3 : SPI_MODE_0; // MCP3201: MODE0 или MODE3 (экв. 1,1)

    uint8_t bits = 8;

    int fd = open(dev, O_RDWR);
    if (fd < 0) die("open spidev");

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) die("SPI_IOC_WR_MODE");
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) die("SPI_IOC_WR_BITS_PER_WORD");
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) == -1) die("SPI_IOC_WR_MAX_SPEED_HZ");

    signal(SIGINT, on_sigint);

    while (!g_stop) {
        uint16_t code = 0;
        if (mcp3201_read12(fd, speed_hz, &code) != 0) die("SPI transfer");

        printf("%u ", (unsigned)code);
	printf("%x\n", (unsigned)code);
        fflush(stdout);

        if (delay_us > 0) usleep((useconds_t)delay_us);
    }

    close(fd);
    return 0;
}

