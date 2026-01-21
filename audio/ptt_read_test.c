/*
 * ptt_read_test.c
 *
 * Минимальный тест: читаем GPIO2_A0 (gpiochip2, line 0)
 * и постоянно выводим его значение.
 *
 * Сборка:
 *   gcc -O2 -Wall ptt_read_test.c -o ptt_read_test -lgpiod
 *
 * Запуск:
 *   sudo ./ptt_read_test
 */

#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define PTT_CHIP "gpiochip0"
#define PTT_LINE 27

int main(void)
{
    struct gpiod_chip *chip;
    struct gpiod_line *line;
    int value;

    chip = gpiod_chip_open_by_name(PTT_CHIP);
    if (!chip) {
        perror("gpiod_chip_open_by_name");
        return 1;
    }

    line = gpiod_chip_get_line(chip, PTT_LINE);
    if (!line) {
        perror("gpiod_chip_get_line");
        return 1;
    }

    if (gpiod_line_request_input(line, "ptt_read_test") < 0) {
        perror("gpiod_line_request_input");
        return 1;
    }

    printf("Reading PTT (GPIO2_A0 = %s line %d)\n", PTT_CHIP, PTT_LINE);
    printf("Ctrl+C to exit\n\n");

    while (1) {
        value = gpiod_line_get_value(line);
        if (value < 0) {
            perror("gpiod_line_get_value");
            break;
        }

        printf("PTT = %d\n", value);
        fflush(stdout);
        usleep(200000); // 200 мс
    }

    gpiod_line_release(line);
    gpiod_chip_close(chip);
    return 0;
}

