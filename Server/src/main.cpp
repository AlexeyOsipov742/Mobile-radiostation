#include "TxRx.h"
#include "uart_screen_forward.h"
#include "uart_buttons_rx.h"
#include <thread>

int main() {
    unsigned char *buffer = (unsigned char *)std::malloc(BUFFER_SIZE);
    if (!buffer) { std::perror("malloc"); return 1; }

    if (!gpio_init()) {
        std::fprintf(stderr, "gpio_init failed\n");
        std::free(buffer);
        return 1;
    }

    gpio_set_ptt(0);
    gpio_set_dac_cs(1);
    gpio_set_adc_cs(1);


    {
    	const int screen_port = 7777;           // как на Pi (debug_client/основная логика)
    	const char* uart_dev  = "/dev/ttyUSB0"; // как у тебя в debug_server
    	const int window_ms   = 300;

    	std::thread(uart_screen_forward_thread, SERVER_IP, screen_port, uart_dev, 9600, window_ms).detach();
    }

    {
        const int buttons_port = 7778;          // Pi -> NaPi (кнопки)  !!! новый порт
        const char* uart_dev   = "/dev/ttyUSB0";
        std::thread(uart_buttons_rx_thread, buttons_port, uart_dev, 9600).detach();
    }

    while (1) {
        // COR active LOW
        if (gpio_get_cor_level() == 0) {
            audioTxEth_PI(buffer);
        } else {
            audioRxEth_PI(buffer);
        }
    }

    gpio_cleanup();
    std::free(buffer);
    return 0;
}
