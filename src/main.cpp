#include "TxRx.h"

int main() {   
    
    short *buffer = (short *)malloc(BUFFER_SIZE * sizeof(short));// Выделение памяти для буфера

    while (1) {
        //RxEth(buffer);
        //Tx(buffer);
        //audioRxEth(buffer);
        audioTxEth(buffer);
        //Rx(buffer);
        //TxEth(buffer);
    }

    return 0;
}