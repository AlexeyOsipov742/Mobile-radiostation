#include "TxRx.h"

int main() {
    unsigned char *buffer = (unsigned char *)std::malloc(BUFFER_SIZE * sizeof(*buffer));
    if (!buffer) {
        std::perror("malloc");
        return 1;
    }

    // BCM numbering
    if (wiringPiSetupGpio() == -1) {
        std::perror("wiringPiSetupGpio");
        std::free(buffer);
        return 1;
    }

    // COR input (active LOW)
    pinMode(RPI_COR_GPIO, INPUT);
    pullUpDnControl(RPI_COR_GPIO, PUD_UP);

    // PTT output (active HIGH)
    pinMode(RPI_PTT_GPIO, OUTPUT);
    digitalWrite(RPI_PTT_GPIO, LOW);

    // CS lines for MCP4822/MCP3201 (active LOW)
    pinMode(RPI_DAC_CS_GPIO, OUTPUT);
    digitalWrite(RPI_DAC_CS_GPIO, HIGH);
    pinMode(RPI_ADC_CS_GPIO, OUTPUT);
    digitalWrite(RPI_ADC_CS_GPIO, HIGH);

    while (1) {
        if (digitalRead(RPI_COR_GPIO) == LOW) {
            // Radio receives (COR active): sample from ADC and stream to NaPi
            audioTxEth_PI(buffer);
        } else {
            // No RX from air: wait for NaPi TX, key PTT and play to DAC
            audioRxEth_PI(buffer);
        }
    }

    std::free(buffer);
    return 0;
}