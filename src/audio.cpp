#include "TxRx.h"

void audio(std::atomic<bool> &running) {
    unsigned char audioBuffer[BUFFER_SIZE];

    if (wiringPiSetupGpio() == -1) {
        perror("GPIO setup failed");
        return ;
    }
    
    int gpio_pin = 19; // GPIO номер для 37 пина на плате
    pinMode(gpio_pin, INPUT); // Настройка пина как вход
    pullUpDnControl(gpio_pin, PUD_UP); // Подтяжка к "земле" для стабильности

    while (running) {
        if (digitalRead(gpio_pin) == LOW) {
            audioTxEth_PI(audioBuffer, running);
        } else {
            audioRxEth_PI(audioBuffer, running);
        }
    }
}