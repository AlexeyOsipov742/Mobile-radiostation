#include "TxRx.h"
#include "uart_screen_forward.h"
#include "uart_buttons_rx.h"
#include <thread>
#include <cstdlib>

static const char* env_or_default(const char* name, const char* fallback) {
    const char* v = std::getenv(name);
    return (v && *v) ? v : fallback;
}

static unsigned env_u32_or_default(const char* name, unsigned fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    char* end = nullptr;
    unsigned long x = std::strtoul(v, &end, 10);
    if (!end || *end != '\0') return fallback;
    return static_cast<unsigned>(x);
}

static bool env_bool_or_default(const char* name, bool fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    if (std::strcmp(v, "1") == 0 || std::strcmp(v, "true") == 0 || std::strcmp(v, "TRUE") == 0 ||
        std::strcmp(v, "yes") == 0 || std::strcmp(v, "YES") == 0) {
        return true;
    }
    if (std::strcmp(v, "0") == 0 || std::strcmp(v, "false") == 0 || std::strcmp(v, "FALSE") == 0 ||
        std::strcmp(v, "no") == 0 || std::strcmp(v, "NO") == 0) {
        return false;
    }
    return fallback;
}

int main() {
    unsigned char *buffer = (unsigned char *)std::malloc(BUFFER_SIZE);
    if (!buffer) { std::perror("malloc"); return 1; }

    if (!gpio_init()) {
        std::fprintf(stderr, "gpio_init failed\n");
        std::free(buffer);
        return 1;
    }

    gpio_set_ptt(0);

    const char* uart_dev       = env_or_default("SB9600_UART_DEV", "/dev/ttyS0");
    const char* sb9600_gpiochip = env_or_default("SB9600_GPIOCHIP", "/dev/gpiochip2");
    const unsigned sb9600_rts_line = env_u32_or_default("SB9600_RTS_LINE", 13u); // GPIO2_B5
    const unsigned sb9600_cts_line = env_u32_or_default("SB9600_CTS_LINE", 14u); // GPIO2_B6
    const bool sb9600_rts_active_low = env_bool_or_default("SB9600_RTS_ACTIVE_LOW", true);
    const bool sb9600_cts_active_low = env_bool_or_default("SB9600_CTS_ACTIVE_LOW", false);
    const bool sb9600_rx_invert = env_bool_or_default("SB9600_RX_INVERT", false);
    const bool sb9600_tx_invert = env_bool_or_default("SB9600_TX_INVERT", false);

    {
    	const int screen_port = 7777;           // как на Pi (debug_client/основная логика)
    	const int window_ms   = 300;

    	std::thread(uart_screen_forward_thread,
                    SERVER_IP,
                    screen_port,
                    uart_dev,
                    9600,
                    window_ms,
                    sb9600_rx_invert).detach();
    }

    {
        const int buttons_port = 7778;          // Pi -> NaPi (кнопки)  !!! новый порт
        std::thread(uart_buttons_rx_thread,
                    buttons_port,
                    uart_dev,
                    9600,
                    sb9600_gpiochip,
                    sb9600_rts_line,
                    sb9600_cts_line,
                    sb9600_rts_active_low,
                    sb9600_cts_active_low,
                    sb9600_tx_invert).detach();
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
