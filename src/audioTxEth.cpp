#include "TxRx.h"

//#define SERVER_IP "192.168.1.112" // IP адрес Митино
#define SERVER_IP "192.168.0.119" // IP адрес дом
//#define SERVER_IP "10.10.1.62"  // IP адрес работа

void audioTxEth(short *buffer) {
    // Параметры для захвата звука
    snd_pcm_t *capture_handle;
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_uframes_t local_buffer = BUFFER_SIZE;
    snd_pcm_uframes_t local_periods = PERIODS;

    // Создание сокета для передачи данных
    int sockfd;
    struct sockaddr_in serv_addr;

    unsigned int resample = 1;
    unsigned int sampleRate = 44100;
    long int dataCapacity = 0;
    int channels = 2;
    
    
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    //serv_addr.sin_addr.s_addr = inet_addr("SERVER_IP"); // IP работа
    serv_addr.sin_addr.s_addr = inet_addr(SERVER_IP); // IP дом

    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection failed");
    }

    //printf("sus1\n");
    // Открываем PCM устройство
    if (snd_pcm_open(&capture_handle, "plughw:0,6", SND_PCM_STREAM_CAPTURE, 0) < 0) {
        perror("Cannot open audio device");
        close(sockfd);
        return;
    }

    //printf("sus2\n");
    // Выделение памяти для hw_params с использованием malloc
    if (snd_pcm_hw_params_malloc(&hw_params) < 0) {
        perror("Cannot allocate hardware parameters");
        snd_pcm_close(capture_handle);
        close(sockfd);
        return;
    }

    //printf("sus3\n");

    if (snd_pcm_hw_params_any(capture_handle, hw_params) < 0) {
        perror("Cannot configure this PCM device");
        snd_pcm_hw_params_free(hw_params);
        snd_pcm_close(capture_handle);
        close(sockfd);
        return;
    }

    snd_pcm_hw_params_get_buffer_size(hw_params, &local_buffer);
    snd_pcm_hw_params_get_period_size(hw_params, &local_periods, 0);

    //printf("Buffer size: %lu, Period size: %lu\n", local_buffer, local_periods);

    if (snd_pcm_hw_params_set_rate_resample(capture_handle, hw_params, resample) < 0) {
        perror("Cannot set sample rate");
        snd_pcm_hw_params_free(hw_params);
        snd_pcm_close(capture_handle);
        close(sockfd);
        return;
    }
    
    if (snd_pcm_hw_params_set_access(capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED) < 0) {
        perror("Cannot set access rate");
        snd_pcm_hw_params_free(hw_params);
        snd_pcm_close(capture_handle);
        close(sockfd);
        return;
    }

    if (snd_pcm_hw_params_set_format(capture_handle, hw_params, SND_PCM_FORMAT_S16_LE) < 0) {
        perror("Cannot set sample format");
        snd_pcm_hw_params_free(hw_params);
        snd_pcm_close(capture_handle);
        close(sockfd);
        return;
    }

    if (snd_pcm_hw_params_set_channels(capture_handle, hw_params, channels) < 0) {
        perror("Cannot set channel count");
        snd_pcm_hw_params_free(hw_params);
        snd_pcm_close(capture_handle);
        close(sockfd);
        return;
    }

    if (snd_pcm_hw_params_set_rate_near(capture_handle, hw_params, &sampleRate, 0) < 0) {
        perror("Cannot set rate near");
        snd_pcm_hw_params_free(hw_params);
        snd_pcm_close(capture_handle);
        close(sockfd);
        return;
    }

    if (snd_pcm_hw_params_set_buffer_size_near(capture_handle, hw_params, &local_buffer) < 0) {
        perror("Cannot set buffer size near");
        snd_pcm_hw_params_free(hw_params);
        snd_pcm_close(capture_handle);
        close(sockfd);
        return;
    }

    if (snd_pcm_hw_params(capture_handle, hw_params) < 0) {
        perror("Cannot set hardware parameters");
        snd_pcm_hw_params_free(hw_params);
        snd_pcm_close(capture_handle);
        close(sockfd);
        return;
    }

    //printf("sus4\n");

    // Освобождение выделенной памяти
    snd_pcm_hw_params_free(hw_params);

    //printf("sus5\n");

    snd_pcm_hw_params_get_buffer_size(hw_params, &local_buffer);
    snd_pcm_hw_params_get_period_size(hw_params, &local_periods, 0);

    printf("Buffer size: %lu, Period size: %lu\n", local_buffer, local_periods);

    // Подготовка устройства к воспроизведению
    if (snd_pcm_prepare(capture_handle) < 0) {
        perror("Cannot prepare audio interface for use");
        snd_pcm_close(capture_handle);
        close(sockfd);
        return;
    }

    //printf("sus6\n");

   // Основной цикл для захвата и передачи данных
    while (1) {
        // Захватываем аудиоданные
        //printf("[FLOPS]: Buffer = `%p` and size = `%llu`;\n", buffer, BUFFER_SIZE / (channels * 2));

        int frames = snd_pcm_readi(capture_handle, buffer, BUFFER_SIZE / (channels * 2));
        //printf("sus6.5\n");
        if (frames < 0) {
            fprintf(stderr, "Read error: %s\n", snd_strerror(frames));  // Выводим точную ошибку ALSA
            snd_pcm_prepare(capture_handle);  // Попробуем восстановить поток
            continue;
        }

        //printf("sus7\n");

        /*for (unsigned int i = 0; i < sampleRate; i++) {
		buffer[i] = 10000 * sinf(2 * M_PI * 100 * ((float)i / sampleRate));
	    }*/                                                                     //Генерация синусоиды

        /*for (int i = 0; i < BUFFER_SIZE; i++) {
            printf("%02x", buffer[i]);
            if (((i + 1) % 16) == 0)
                printf("\n");
        }                                           //Отладка
    */
        // Передаем данные по сети
        //ssize_t bytes_sent = send(sockfd, buffer, frames * channels * 2, 0);
        ssize_t bytes_sent = send(sockfd, buffer,BUFFER_SIZE / (channels * 2), 0);  //Если синусоида, то просто BUFFER_SIZE
        if (bytes_sent < 0) {
            perror("Send error");
            break;
        }
        dataCapacity += BUFFER_SIZE;
        printf("\ndataCapacity: %ld\n\n", dataCapacity);
        
    }

    snd_pcm_drop(capture_handle);
    snd_pcm_close(capture_handle);
    close(sockfd);
}