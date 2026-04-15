#include "TxRx.h"

std::atomic<bool> audio_running(true);
std::atomic<bool> cmd_running(true);
std::atomic<uint64_t> g_cmd_audio_mute_until_ns(0);

uint64_t monotonic_now_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

void audio_mute_for_cmd_ms(uint32_t ms) {
    const uint64_t now = monotonic_now_ns();
    const uint64_t until = now + (uint64_t)ms * 1000000ull;

    uint64_t cur = g_cmd_audio_mute_until_ns.load(std::memory_order_relaxed);
    while (cur < until &&
           !g_cmd_audio_mute_until_ns.compare_exchange_weak(
               cur, until, std::memory_order_release, std::memory_order_relaxed)) {
    }
}

void audio_mute_set_for_cmd_ms(uint32_t ms) {
    const uint64_t now = monotonic_now_ns();
    const uint64_t until = now + (uint64_t)ms * 1000000ull;
    g_cmd_audio_mute_until_ns.store(until, std::memory_order_release);
}

void audio_unmute_cmd_now() {
    g_cmd_audio_mute_until_ns.store(0, std::memory_order_release);
}

bool audio_is_muted_for_cmd() {
    return monotonic_now_ns() < g_cmd_audio_mute_until_ns.load(std::memory_order_acquire);
}

void signal_handler(int signal) {
    std::cout << "\nCtrl+C detected. Stopping threads...\n";
    audio_running = false;
    cmd_running = false;
}
