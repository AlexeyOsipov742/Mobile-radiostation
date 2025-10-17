#include "TxRx.h"

#include <atomic>
#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr const char *kSpiAdcDevice = "/dev/spidev1.0";
constexpr bool kAdcChannelB = false;
constexpr uint32_t kRecordSampleRate = 44'100u;
constexpr float kRecordGain = 1.0f;
constexpr size_t kSamplesPerPacket = BUFFER_SIZE / sizeof(int16_t);
constexpr uint32_t kRetryDelayUsec = 10'000u;
constexpr float kPi = 3.14159265358979323846f;

struct HighPassFilter {
    float alpha{};
    float prevInput{};
    float prevOutput{};
};

void hpfInit(HighPassFilter &f, float sampleRate, float cutoff) {
    float alpha = std::exp(-2.0f * kPi * cutoff / sampleRate);
    f.alpha = alpha;
    f.prevInput = 0.0f;
    f.prevOutput = 0.0f;
}

float hpfRun(HighPassFilter &f, float input) {
    float output = (1.0f - f.alpha) * (input - f.prevInput) + f.alpha * f.prevOutput;
    f.prevInput = input;
    f.prevOutput = output;
    return output;
}

inline void buildCommandTriplet(uint8_t *dst, bool channelB) {
    dst[0] = 0x01;
    dst[1] = static_cast<uint8_t>(0xA0 | ((channelB ? 1 : 0) << 6));
    dst[2] = 0x00;
}

inline uint16_t parseU12(const uint8_t *src) {
    return static_cast<uint16_t>(((src[1] & 0x0F) << 8) | src[2]);
}

int openSpi(const char *device, uint32_t &hz) {
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("open spidev");
        return -1;
    }

    uint8_t mode = SPI_MODE_0;
    uint8_t bpw = 8;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) {
        perror("SPI_IOC_WR_MODE");
        close(fd);
        return -1;
    }
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) == -1) {
        perror("SPI_IOC_WR_BITS_PER_WORD");
        close(fd);
        return -1;
    }

    uint32_t requested = hz;
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &requested) == -1) {
        perror("SPI_IOC_WR_MAX_SPEED_HZ");
        close(fd);
        return -1;
    }

    uint32_t actual = 0;
    if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &actual) == -1) {
        actual = requested;
    }
    if (actual && actual != hz) {
        fprintf(stderr, "WARN: requested %u Hz, driver %u Hz\n", hz, actual);
    }
    if (actual) {
        hz = actual;
    }
    return fd;
}

bool sendAll(int sockfd, const uint8_t *data, size_t bytes) {
    size_t sent = 0;
    while (sent < bytes) {
        ssize_t chunk = send(sockfd, data + sent, bytes - sent, 0);
        if (chunk < 0) {
            if (errno == EINTR) {
                continue;
            }
            perror("Send error");
            return false;
        }
        if (chunk == 0) {
            return false;
        }
        sent += static_cast<size_t>(chunk);
    }
    return true;
}

}  // namespace

void audioTxEth_PI(unsigned char *buffer, std::atomic<bool> &running) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("Socket creation error");
        return;
    }

    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    serv_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

    if (connect(sockfd, reinterpret_cast<sockaddr *>(&serv_addr), sizeof(serv_addr)) < 0) {
        perror("Connection failed");
        close(sockfd);
        return;
    }

    uint32_t spi_hz = kRecordSampleRate * 24;
    int spi_fd = openSpi(kSpiAdcDevice, spi_hz);
    if (spi_fd < 0) {
        close(sockfd);
        return;
    }
    fprintf(stderr, "SPI (ADC) speed set: %u Hz\n", spi_hz);

    const size_t txrx_len = kSamplesPerPacket * 3;
    std::vector<uint8_t> tx(txrx_len);
    std::vector<uint8_t> rx(txrx_len);
    for (size_t i = 0; i < kSamplesPerPacket; ++i) {
        buildCommandTriplet(tx.data() + i * 3, kAdcChannelB);
    }

    spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    tr.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    tr.len = static_cast<uint32_t>(txrx_len);
    tr.speed_hz = spi_hz;
    tr.bits_per_word = 8;

    HighPassFilter hpf{};
    hpfInit(hpf, static_cast<float>(kRecordSampleRate), 20.0f);

    std::vector<int16_t> pcm(kSamplesPerPacket);

    while (running) {
        if (gpio_get_ptt_level() != 0) {
            break;
        }
        if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            if (errno == EINTR) {
                continue;
            }
            perror("SPI_IOC_MESSAGE");
            break;
        }

        for (size_t i = 0; i < kSamplesPerPacket; ++i) {
            uint16_t u12 = parseU12(rx.data() + i * 3);
            float normalized = (static_cast<int>(u12) - 2048) * (1.0f / 2048.0f);
            float filtered = hpfRun(hpf, normalized) * kRecordGain;
            if (filtered > 0.999f) filtered = 0.999f;
            if (filtered < -0.999f) filtered = -0.999f;
            pcm[i] = static_cast<int16_t>(std::lrintf(filtered * 32767.0f));
        }

        if (!sendAll(sockfd, reinterpret_cast<uint8_t *>(pcm.data()), kSamplesPerPacket * sizeof(int16_t))) {
            break;
        }
    }

    close(spi_fd);
    close(sockfd);
    memset(buffer, 0, BUFFER_SIZE);
    usleep(kRetryDelayUsec);
}
