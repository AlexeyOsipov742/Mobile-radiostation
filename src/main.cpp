#include "TxRx.h"

int main() {   
    unsigned char buffer[BUFFER_SIZE]; // Выделение памяти для буфера

    while (1) {
        memset(buffer, 0, BUFFER_SIZE);
        
        RxEth(buffer);
        Tx(buffer);
        //audioRxEth(buffer);
        //audioTxEth(buffer);
        Rx(buffer);
        TxEth(buffer);
        
    }

    return 0;
}