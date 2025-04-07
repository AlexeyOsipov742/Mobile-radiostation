#include "TxRx.h"

void command(std::atomic<bool> &running) {
    unsigned char cmdBuffer[2048];
    while (running) {
        int received = RxEth(cmdBuffer);  // возвращает кол-во байт или 0, если нет данных

        if (received > 0) {
            Tx(cmdBuffer);
	        usleep(10000);
            Rx(cmdBuffer);
            TxEth(cmdBuffer);
            memset(cmdBuffer, 0, sizeof(cmdBuffer));
        }

        usleep(10000);  // чтобы не грузить CPU
    }

   
}
