#include "TxRx.h"

std::atomic<bool> audio_running(true);
std::atomic<bool> cmd_running(true);

void signal_handler(int signal) {
    std::cout << "\nCtrl+C detected. Stopping threads...\n";
    audio_running = false;
    cmd_running = false;
}
