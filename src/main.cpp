#include "TxRx.h"
#include <wiringPi.h>

int main() {   
    
    unsigned char *buffer = (unsigned char *)malloc(BUFFER_SIZE * sizeof(*buffer));// Выделение памяти для буфера

    if (wiringPiSetupGpio() == -1) {
        perror("GPIO setup failed");
        return 0;
    }

    int gpio_pin = 19; // GPIO номер для 37 пина на плате
    pinMode(gpio_pin, INPUT); // Настройка пина как вход
    pullUpDnControl(gpio_pin, PUD_UP); // Подтяжка к "земле" для стабильности
    
    while(1) {
        if (digitalRead(gpio_pin) == LOW) {
            audioTxEth_PI(buffer);
        } else {
            audioRxEth_PI(buffer);
        }
    }
    /*while(1) {
        RxEth(buffer);
        Tx(buffer);
        Rx(buffer);
        TxEth(buffer);
    }*/
    free(buffer);

    return 0;
}