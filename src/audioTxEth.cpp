#include "TxRx.h"

#define SAMPLE_RATE 44100

void audioTxEth(unsigned char *buffer) {
    // Параметры для захвата звука
    snd_pcm_t *capture_handle;
    snd_pcm_hw_params_t *hw_params;

    // Создание сокета для передачи данных
    int sockfd;
    struct sockaddr_in serv_addr;

    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(5678);
    serv_addr.sin_addr.s_addr = inet_addr("10.10.1.62"); // IP адрес Raspberry Pi

    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection failed");
    }

    // Открываем PCM устройство
    if (snd_pcm_open(&capture_handle, "default", SND_PCM_STREAM_CAPTURE, 0) < 0) {
        perror("Cannot open audio device");
    }

    snd_pcm_hw_params_alloca(&hw_params);

    if (snd_pcm_hw_params_any(capture_handle, hw_params) < 0) {
        perror("Cannot configure this PCM device");
    }

    if (snd_pcm_hw_params_set_format(capture_handle, hw_params, SND_PCM_FORMAT_S16_LE) < 0) {
        perror("Cannot set sample format");
    }

    if (snd_pcm_hw_params_set_rate(capture_handle, hw_params, SAMPLE_RATE, 0) < 0) {
        perror("Cannot set sample rate");
    }

    if (snd_pcm_hw_params_set_channels(capture_handle, hw_params, 2) < 0) {
        perror("Cannot set channel count");
    }

    if (snd_pcm_hw_params(capture_handle, hw_params) < 0) {
        perror("Cannot set hardware parameters");
    }

   // Основной цикл для захвата и передачи данных
    while (1) {
        // Захватываем аудиоданные
        int frames = snd_pcm_readi(capture_handle, buffer, BUFFER_SIZE / 4);  // 1024 байт = 256 фреймов при стерео 16-битном аудио
        if (frames < 0) {
            fprintf(stderr, "Read error: %s\n", snd_strerror(frames));  // Выводим точную ошибку ALSA
            snd_pcm_prepare(capture_handle);  // Попробуем восстановить поток
            continue;
        }

        // Передаем данные по сети
        ssize_t bytes_sent = send(sockfd, buffer, frames * 4, 0);  // Отправляем реальные данные (фреймы * 4 байта)
        if (bytes_sent < 0) {
            perror("Send error");
            break;
        }
    }

    snd_pcm_close(capture_handle);
    close(sockfd);
}