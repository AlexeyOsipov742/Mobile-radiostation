#include "TxRx.h"
#include <cerrno>
#include <cstdio>
#include <cstring>

static gpiod_chip* g_chip = nullptr;

static gpiod_line* g_cor = nullptr;
static gpiod_line* g_ptt = nullptr;
static gpiod_line* g_dac_cs = nullptr;
static gpiod_line* g_adc_cs = nullptr;

static bool req_in(gpiod_line* line, const char* name) {
  if (!line) return false;
  if (gpiod_line_request_input(line, name) != 0) {
    std::fprintf(stderr, "[GPIO] request_input failed: %s\n", std::strerror(errno));
    return false;
  }
  return true;
}

static bool req_out(gpiod_line* line, const char* name, int init) {
  if (!line) return false;
  if (gpiod_line_request_output(line, name, init) != 0) {
    std::fprintf(stderr, "[GPIO] request_output failed: %s\n", std::strerror(errno));
    return false;
  }
  return true;
}

bool gpio_init() {
  g_chip = gpiod_chip_open(RPI_GPIO_CHIP);
  if (!g_chip) {
    std::fprintf(stderr, "[GPIO] gpiod_chip_open(%s) failed: %s\n", RPI_GPIO_CHIP, std::strerror(errno));
    return false;
  }

  g_cor    = gpiod_chip_get_line(g_chip, RPI_COR_GPIO);
  g_ptt    = gpiod_chip_get_line(g_chip, RPI_PTT_GPIO);
  g_dac_cs = gpiod_chip_get_line(g_chip, RPI_DAC_CS_GPIO);
  g_adc_cs = gpiod_chip_get_line(g_chip, RPI_ADC_CS_GPIO);


  if (!g_cor || !g_ptt || !g_dac_cs || !g_adc_cs) {
    std::fprintf(stderr, "[GPIO] get_line failed (cor/ptt/cs)\n");
    gpio_cleanup();
    return false;
  }

  // COR input
  if (!req_in(g_cor, "rpi-cor")) { gpio_cleanup(); return false; }

  // PTT output (active HIGH, стартуем LOW)
  if (!req_out(g_ptt, "rpi-ptt", 0)) { gpio_cleanup(); return false; }

  // CS outputs (active LOW, держим HIGH)
  if (!req_out(g_dac_cs, "rpi-dac-cs", 1)) { gpio_cleanup(); return false; }
  if (!req_out(g_adc_cs, "rpi-adc-cs", 1)) { gpio_cleanup(); return false; }

  return true;
}

void gpio_cleanup() {
  if (g_cor) { gpiod_line_release(g_cor); g_cor = nullptr; }
  if (g_ptt) { gpiod_line_set_value(g_ptt, 0); gpiod_line_release(g_ptt); g_ptt = nullptr; }
  if (g_dac_cs) { gpiod_line_set_value(g_dac_cs, 1); gpiod_line_release(g_dac_cs); g_dac_cs = nullptr; }
  if (g_adc_cs) { gpiod_line_set_value(g_adc_cs, 1); gpiod_line_release(g_adc_cs); g_adc_cs = nullptr; }

  if (g_chip) { gpiod_chip_close(g_chip); g_chip = nullptr; }
}

int gpio_get_cor_level() {
  if (!g_cor) return 1; // безопаснее считать "COR не активен"
  int v = gpiod_line_get_value(g_cor);
  if (v < 0) return 1;
  return v;
}

void gpio_set_ptt(int level) {
  if (!g_ptt) return;
  (void)gpiod_line_set_value(g_ptt, level ? 1 : 0);
}

void gpio_set_dac_cs(int level) {
  if (!g_dac_cs) return;
  (void)gpiod_line_set_value(g_dac_cs, level ? 1 : 0);
}

void gpio_set_adc_cs(int level) {
  if (!g_adc_cs) return;
  (void)gpiod_line_set_value(g_adc_cs, level ? 1 : 0);
}
