#include "TxRx.h"
#include <alsa/asoundlib.h>

#define SAMPLE_RATE 44100
#define CHANNELS 2

void audioRxEth(unsigned char *buffer) {
    int sockfd, newsockfd;
    struct sockaddr_in serv_addr, cli_addr;
    socklen_t clilen;
    
    snd_pcm_t *playback_handle;
    snd_pcm_hw_params_t *hw_params;

    // Настройка сокета
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
        return;
    }

    int optval = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(5678);

    if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Bind error");
        return;
    }

    listen(sockfd, 5);
    clilen = sizeof(cli_addr);

    if ((newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen)) < 0) {
        perror("Accept error");
        return;
    }

    // Настройка ALSA для воспроизведения
    if (snd_pcm_open(&playback_handle, "hw:1,0", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        perror("Cannot open audio playback device");
        return;
    }

    snd_pcm_hw_params_alloca(&hw_params);
    snd_pcm_hw_params_any(playback_handle, hw_params);
    snd_pcm_hw_params_set_access(playback_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(playback_handle, hw_params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_rate(playback_handle, hw_params, SAMPLE_RATE, 0);
    snd_pcm_hw_params_set_channels(playback_handle, hw_params, CHANNELS);

    if (snd_pcm_hw_params(playback_handle, hw_params) < 0) {
        perror("Cannot set hardware parameters");
        return;
    }

    // Основной цикл для приёма и воспроизведения звуковых данных
    while (1) {
        int n = recv(newsockfd, buffer, BUFFER_SIZE, 0);
        if (n <= 0) {
            if (n == 0) {
                printf("Connection closed by client\n");
            } else {
                perror("Receive error");
            }
            break;
        }

        // Воспроизводим данные с помощью ALSA
        if (snd_pcm_writei(playback_handle, buffer, 256) < 0) {
            snd_pcm_prepare(playback_handle);  // Восстанавливаем поток в случае ошибки
        }
    }

    // Освобождаем ресурсы
    snd_pcm_drain(playback_handle);
    snd_pcm_close(playback_handle);
    close(newsockfd);
    close(sockfd);
}
