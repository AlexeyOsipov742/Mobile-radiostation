#include "TxRx.h"
#include <wiringPi.h>

//std::mutex uart_mutex;

int main() {
    signal(SIGINT, signal_handler);

    std::thread audioThread(audio, std::ref(audio_running));
    std::thread cmdThread(command, std::ref(cmd_running));

    audioThread.join();
    cmdThread.join();

    std::cout << "All threads finished.\n";
    return 0;
}