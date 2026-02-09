// src/buttons_client.cpp
// Integrated buttons client (RPi): MCP23017 IRQ -> GPIO line -> send SB9600 5-byte frames to NaPi via TxEthN()
// Behavior: on PRESSED send "press" frame; on RELEASED send "release" frame (if defined).
//
// Defaults are hardcoded for your setup:
//   I2C: /dev/i2c-1 addr 0x26
//   IRQ: gpiochip0 line 26 (falling edge, MCP INT active-low)
//   TCP port: 7778
//
// This file provides:  void buttons_client(std::atomic<bool>& running);

#include "TxRx.h"
#include "buttons_client.h"
#include <gpiod.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <optional>

// ---- I2C helpers ----
static int i2c_write_reg(int fd, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return (write(fd, buf, 2) == 2) ? 0 : -1;
}
static int i2c_read_reg2(int fd, uint8_t reg, uint8_t *a, uint8_t *b) {
    if (write(fd, &reg, 1) != 1) return -1;
    uint8_t buf[2];
    if (read(fd, buf, 2) != 2) return -1;
    *a = buf[0]; *b = buf[1];
    return 0;
}

// ---- MCP23017 registers (BANK=0) ----
enum : uint8_t {
    IODIRA   = 0x00,
    IODIRB   = 0x01,
    GPINTENA = 0x04,
    GPINTENB = 0x05,
    INTCONA  = 0x08,
    INTCONB  = 0x09,
    IOCON    = 0x0A, // also at 0x0B
    GPPUA    = 0x0C,
    GPPUB    = 0x0D,
    INTFA    = 0x0E,
    INTFB    = 0x0F,
    INTCAPA  = 0x10,
    INTCAPB  = 0x11,
};

static void hex_dump5(const uint8_t* p) {
    std::printf("%02X %02X %02X %02X %02X", p[0], p[1], p[2], p[3], p[4]);
}

struct Cmd5 { std::array<uint8_t,5> b; };

struct ButtonAction {
    // what to send on press/release
    std::optional<Cmd5> press;
    std::optional<Cmd5> release;
};

// Pick "press" as SHORT if present, else LONG (if present), else only-press
static std::optional<ButtonAction> map_pin_to_action(int pin) {
    ButtonAction a{};

    switch (pin) {
        // BUTTON 1
        // LONG  : 05 83 01 57 C1
        // SHORT : 05 83 02 57 7A
        // REL   : 05 83 00 57 A8
        case 0:
            a.press   = Cmd5{{ {0x05,0x83,0x02,0x57,0x7A} }}; // SHORT by default
            a.release = Cmd5{{ {0x05,0x83,0x00,0x57,0xA8} }};
            return a;

        // BUTTON 2
        case 1:
            a.press   = Cmd5{{ {0x05,0x89,0x02,0x57,0x24} }}; // SHORT
            a.release = Cmd5{{ {0x05,0x89,0x00,0x57,0xF6} }};
            return a;

        // BUTTON 3
        // LONG  : 05 84 01 57 0E
        // SHORT : 05 84 02 57 B5
        // release in your table is unclear (you had "PUSH DOWN 05 84 02 57 67" which is NOT 00-form).
        // So: send press only (SHORT). If later you confirm real release, add it here.
        case 2:
            a.press = Cmd5{{ {0x05,0x84,0x02,0x57,0xB5} }};  // SHORT
            // a.release = Cmd5{{ {0x05,0x84,0x00,0x57,0x??} }};
            return a;

        // BUTTON 4
        case 3:
            a.press   = Cmd5{{ {0x05,0x8A,0x02,0x57,0xB4} }}; // SHORT
            a.release = Cmd5{{ {0x05,0x8A,0x00,0x57,0x66} }};
            return a;

        // BUTTON 5
        case 4:
            a.press   = Cmd5{{ {0x05,0x85,0x02,0x57,0xC5} }}; // SHORT
            a.release = Cmd5{{ {0x05,0x85,0x00,0x57,0x17} }};
            return a;

        // JOYSTICK LEFT
        case 5:
            a.press   = Cmd5{{ {0x05,0x80,0x02,0x57,0xEA} }}; // SHORT
            a.release = Cmd5{{ {0x05,0x80,0x00,0x57,0x38} }};
            return a;

        // JOYSTICK RIGHT
        case 6:
            a.press   = Cmd5{{ {0x05,0x82,0x02,0x57,0x0A} }}; // SHORT
            a.release = Cmd5{{ {0x05,0x82,0x00,0x57,0xD8} }};
            return a;

        // HOME (only press)
        case 7:
            a.press = Cmd5{{ {0x05,0x81,0x01,0x57,0x21} }};
            return a;

        // Add more pins here as you wire them:
        // PTT:
        // a.press   = Cmd5{{ {0x05,0x01,0x01,0x57,0xDA} }};
        // a.release = Cmd5{{ {0x05,0x01,0x00,0x57,0xB3} }};

        default:
            return std::nullopt;
    }
}

static bool setup_mcp23017_irq(int fd) {
    // IOCON: MIRROR=1 (bit6), ODR=1 (bit2), INTPOL=0 (active-low)
    uint8_t iocon = 0x40 | 0x04; // 0x44

    if (i2c_write_reg(fd, IOCON,   iocon) < 0) return false;
    if (i2c_write_reg(fd, IOCON+1, iocon) < 0) return false;

    // inputs
    if (i2c_write_reg(fd, IODIRA, 0xFF) < 0) return false;
    if (i2c_write_reg(fd, IODIRB, 0xFF) < 0) return false;

    // pull-up
    if (i2c_write_reg(fd, GPPUA, 0xFF) < 0) return false;
    if (i2c_write_reg(fd, GPPUB, 0xFF) < 0) return false;

    // IRQ on change (INTCON=0)
    if (i2c_write_reg(fd, INTCONA, 0x00) < 0) return false;
    if (i2c_write_reg(fd, INTCONB, 0x00) < 0) return false;

    // enable IRQ on all pins
    if (i2c_write_reg(fd, GPINTENA, 0xFF) < 0) return false;
    if (i2c_write_reg(fd, GPINTENB, 0xFF) < 0) return false;

    // clear any pending IRQ
    uint8_t dumpA=0, dumpB=0;
    if (i2c_read_reg2(fd, INTCAPA, &dumpA, &dumpB) < 0) return false;
    (void)dumpA; (void)dumpB;

    return true;
}

void buttons_client(std::atomic<bool>& running) {
    // ---- defaults (your setup) ----
    const char* i2c_dev   = "/dev/i2c-1";
    const int   i2c_addr  = 0x26;
    const char* gpiochip  = "gpiochip0";
    const int   irq_line  = 26;
    const int   tcp_port  = 7778;

    // --- open i2c ---
    int fd = ::open(i2c_dev, O_RDWR);
    if (fd < 0) { std::perror("[BTN] open i2c"); return; }
    if (ioctl(fd, I2C_SLAVE, i2c_addr) < 0) { std::perror("[BTN] ioctl I2C_SLAVE"); ::close(fd); return; }

    if (!setup_mcp23017_irq(fd)) {
        std::fprintf(stderr, "[BTN] Failed to setup MCP23017 IRQ: %s\n", std::strerror(errno));
        ::close(fd);
        return;
    }

    // --- gpio IRQ line ---
    gpiod_chip* chip = gpiod_chip_open_by_name(gpiochip);
    if (!chip) { std::perror("[BTN] gpiod_chip_open_by_name"); ::close(fd); return; }

    gpiod_line* line = gpiod_chip_get_line(chip, irq_line);
    if (!line) { std::perror("[BTN] gpiod_chip_get_line"); gpiod_chip_close(chip); ::close(fd); return; }

    gpiod_line_request_config cfg{};
    cfg.consumer = "buttons_client";
    cfg.request_type = GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE; // MCP INT active-low
    cfg.flags = 0;
#ifdef GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP
    cfg.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
#endif

    if (gpiod_line_request(line, &cfg, 0) < 0) {
        std::perror("[BTN] gpiod_line_request");
        gpiod_chip_close(chip);
        ::close(fd);
        return;
    }

    std::printf("[BTN] MCP@0x%02X %s, IRQ on %s:%d -> NaPi:%d\n",
                i2c_addr, i2c_dev, gpiochip, irq_line, tcp_port);
    std::printf("[BTN] Send press on PRESSED, release on released (if mapped)\n");
    std::fflush(stdout);

    // debounce presses only (not releases)
    std::array<uint64_t,16> last_press_ms{};
    auto now_ms = []() -> uint64_t {
        timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (uint64_t)ts.tv_sec*1000ULL + (uint64_t)ts.tv_nsec/1000000ULL;
    };

    while (running.load()) {
        // Use timeout so thread can exit quickly when running=false
        timespec to{};
        to.tv_sec = 0;
        to.tv_nsec = 200 * 1000 * 1000; // 200ms

        int rv = gpiod_line_event_wait(line, &to);
        if (!running.load()) break;
        if (rv < 0) { std::perror("[BTN] gpiod_line_event_wait"); break; }
        if (rv == 0) continue;

        gpiod_line_event ev{};
        if (gpiod_line_event_read(line, &ev) < 0) { std::perror("[BTN] gpiod_line_event_read"); break; }

        uint8_t intfA=0, intfB=0, capA=0, capB=0;
        if (i2c_read_reg2(fd, INTFA, &intfA, &intfB) < 0) { std::perror("[BTN] read INTF"); break; }
        if (i2c_read_reg2(fd, INTCAPA, &capA, &capB) < 0) { std::perror("[BTN] read INTCAP"); break; }

        // Port A
        for (int bit = 0; bit < 8; bit++) {
            if (!(intfA & (1u<<bit))) continue;

            int pin = bit; // 0..7
            int level = (capA & (1u<<bit)) ? 1 : 0;
            bool pressed = (level == 0);

            auto act = map_pin_to_action(pin);
            if (!act) continue;

            if (pressed) {
                uint64_t t = now_ms();
                if (t - last_press_ms[pin] < 30) continue;
                last_press_ms[pin] = t;

                if (act->press) {
                    std::printf("[BTN] pin %d PRESSED -> ", pin);
                    hex_dump5(act->press->b.data());
                    std::printf("\n");
                    (void)TxEthN(act->press->b.data(), 5, tcp_port);
                }
            } else {
                if (act->release) {
                    std::printf("[BTN] pin %d released -> ", pin);
                    hex_dump5(act->release->b.data());
                    std::printf("\n");
                    (void)TxEthN(act->release->b.data(), 5, tcp_port);
                }
            }
        }

        // Port B
        for (int bit = 0; bit < 8; bit++) {
            if (!(intfB & (1u<<bit))) continue;

            int pin = bit + 8; // 8..15
            int level = (capB & (1u<<bit)) ? 1 : 0;
            bool pressed = (level == 0);

            auto act = map_pin_to_action(pin);
            if (!act) continue;

            if (pressed) {
                uint64_t t = now_ms();
                if (t - last_press_ms[pin] < 30) continue;
                last_press_ms[pin] = t;

                if (act->press) {
                    std::printf("[BTN] pin %d PRESSED -> ", pin);
                    hex_dump5(act->press->b.data());
                    std::printf("\n");
                    (void)TxEthN(act->press->b.data(), 5, tcp_port);
                }
            } else {
                if (act->release) {
                    std::printf("[BTN] pin %d released -> ", pin);
                    hex_dump5(act->release->b.data());
                    std::printf("\n");
                    (void)TxEthN(act->release->b.data(), 5, tcp_port);
                    }
            }
        }

        std::fflush(stdout);
    }

    gpiod_line_release(line);
    gpiod_chip_close(chip);
    ::close(fd);

    std::printf("[BTN] stopped\n");
    std::fflush(stdout);
}

