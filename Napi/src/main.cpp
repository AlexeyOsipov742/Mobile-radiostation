#include "TxRx.h"

//std::mutex uart_mutex;

int main() {
    if (!gpio_init()) {
        std::cerr << "Failed to initialize GPIO. Exiting.\n";
        return 1;
    }

    signal(SIGINT, signal_handler);

    std::thread audioThread(audio, std::ref(audio_running));
    std::thread cmdThread(command, std::ref(cmd_running));

    audioThread.join();
    cmdThread.join();

    gpio_cleanup();

    std::cout << "All threads finished.\n";
    return 0;
}
