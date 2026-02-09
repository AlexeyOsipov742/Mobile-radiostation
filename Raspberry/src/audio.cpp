#include "TxRx.h"
#include <time.h>

static uint64_t now_ms() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
}

void audio(std::atomic<bool> &running) {
    unsigned char audioBuffer[BUFFER_SIZE];

    // фильтр дребезга
    int stable = gpio_get_ptt_level();      // 1=RX, 0=TX
    int last_raw = stable;
    uint64_t last_change_ms = now_ms();
    const uint64_t debounce_ms = 30;

    while (running) {
        int raw = gpio_get_ptt_level();
        if (raw != last_raw) {
            last_raw = raw;
            last_change_ms = now_ms();
        }

        if (raw != stable) {
            if (now_ms() - last_change_ms >= debounce_ms) {
                stable = raw;
            }
        }

        if (stable == 0) {
            audioTxEth_client(audioBuffer, running);   // выйдет, когда PTT отпустят
        } else {
            audioRxEth_client(audioBuffer, running);   // выйдет, когда PTT нажмут (внутри есть проверка)
        }
    }
}
