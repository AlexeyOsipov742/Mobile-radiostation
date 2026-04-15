#include "TxRx.h"

#if __has_include(<gpiod.h>)
#include <gpiod.h>
#elif __has_include(<gpiod/gpiod.h>)
#include <gpiod/gpiod.h>
#else
extern "C" {
struct gpiod_chip;
struct gpiod_line;

typedef struct gpiod_chip gpiod_chip;
typedef struct gpiod_line gpiod_line;

gpiod_chip *gpiod_chip_open(const char *path);
void gpiod_chip_close(gpiod_chip *chip);
gpiod_line *gpiod_chip_get_line(gpiod_chip *chip, unsigned int offset);
int gpiod_line_request_input(gpiod_line *line, const char *consumer);
int gpiod_line_request_output(gpiod_line *line, const char *consumer, int default_val);
void gpiod_line_release(gpiod_line *line);
int gpiod_line_set_value(gpiod_line *line, int value);
int gpiod_line_get_value(gpiod_line *line);
}
#endif
#include <mutex>

namespace {
constexpr const char *kGpioChipPath = "/dev/gpiochip0";
constexpr unsigned int kPttLineNumber = 16;
constexpr unsigned int kLedLineNumber = 20;
constexpr const char *kConsumerName = "mobile-radiostation";

std::mutex gpio_mutex;
gpiod_chip *gpio_chip = nullptr;
gpiod_line *ptt_line = nullptr;
gpiod_line *led_line = nullptr;
}  // namespace

static void gpio_release_locked() {
    if (ptt_line) {
        gpiod_line_release(ptt_line);
        ptt_line = nullptr;
    }
    if (led_line) {
        gpiod_line_set_value(led_line, 0);
        gpiod_line_release(led_line);
        led_line = nullptr;
    }
    if (gpio_chip) {
        gpiod_chip_close(gpio_chip);
        gpio_chip = nullptr;
    }
}

bool gpio_init() {
    std::lock_guard<std::mutex> lock(gpio_mutex);
    if (gpio_chip) {
        return true;
    }

    gpio_chip = gpiod_chip_open(kGpioChipPath);
    if (!gpio_chip) {
        perror("gpiod_chip_open");
        return false;
    }

    ptt_line = gpiod_chip_get_line(gpio_chip, kPttLineNumber);
    if (!ptt_line) {
        perror("gpiod_chip_get_line (PTT)");
        gpio_release_locked();
        return false;
    }
    if (gpiod_line_request_input(
            ptt_line,
            kConsumerName) < 0) {
        perror("gpiod_line_request_input (PTT)");
        gpio_release_locked();
        return false;
    }

    led_line = gpiod_chip_get_line(gpio_chip, kLedLineNumber);
    if (led_line) {
        if (gpiod_line_request_output(
                led_line,
                kConsumerName,
                0) < 0) {
            perror("gpiod_line_request_output (LED)");
            gpiod_line_release(led_line);
            led_line = nullptr;
        }
    } else {
        perror("gpiod_chip_get_line (LED)");
    }

    return true;
}

void gpio_cleanup() {
    std::lock_guard<std::mutex> lock(gpio_mutex);
    gpio_release_locked();
}

int gpio_get_ptt_level() {
    std::lock_guard<std::mutex> lock(gpio_mutex);
    if (!ptt_line) {
        return 1;
    }
    int value = gpiod_line_get_value(ptt_line);
    if (value < 0) {
        perror("gpiod_line_get_value (PTT)");
        return 1;
    }
    return value;
}

void gpio_set_activity_led(bool on) {
    std::lock_guard<std::mutex> lock(gpio_mutex);
    if (!led_line) {
        return;
    }
    if (gpiod_line_set_value(led_line, on ? 1 : 0) < 0) {
        perror("gpiod_line_set_value (LED)");
    }
}
