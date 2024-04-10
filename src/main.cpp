#include "TxRx.h"

int main() {
    while (1) {
        
        RxEth();
        Tx();
        sleep(1);
        Rx();
        TxEth();
        
    }


    return 0;
}