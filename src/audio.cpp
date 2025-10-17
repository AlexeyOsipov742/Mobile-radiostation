#include "TxRx.h"

void audio(std::atomic<bool> &running) {
    unsigned char audioBuffer[BUFFER_SIZE];

    while (running) {
        if (gpio_get_ptt_level() == 0) {
            audioTxEth_PI(audioBuffer, running);
        } else {
            audioRxEth_PI(audioBuffer, running);
        }
    }
}
