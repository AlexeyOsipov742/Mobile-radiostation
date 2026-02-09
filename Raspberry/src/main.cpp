#include "TxRx.h"
#include "buttons_client.h"  // ты положил в include

int main() {
    if (!gpio_init()) {
        std::cerr << "Failed to initialize GPIO. Exiting.\n";
        return 1;
    }

    signal(SIGINT, signal_handler);

    std::thread audioThread(audio, std::ref(audio_running));
    std::thread cmdThread(command, std::ref(cmd_running));

    // Кнопки останавливаем тем же cmd_running (SIGINT уже должен его сбрасыват�

    std::thread btnThread(buttons_client, std::ref(cmd_running));
    audioThread.join();
    cmdThread.join();
    btnThread.join();

    gpio_cleanup();

    std::cout << "All threads finished.\n";
    return 0;
}

