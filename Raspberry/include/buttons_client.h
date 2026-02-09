#pragma once
#include <atomic>

struct ButtonsClientCfg {
  const char* i2c_dev;     // "/dev/i2c-1"
  int i2c_addr;            // 0x26
  const char* gpiochip;    // "gpiochip0"
  int gpio_line;           // 26
  int tcp_port;            // 7778
};

void buttons_client(std::atomic<bool>& run);

