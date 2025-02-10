#include "TxRx.h"
#include <wiringPi.h>

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

int main() {   
    
    unsigned char *buffer = (unsigned char *)malloc(BUFFER_SIZE * sizeof(*buffer));// Выделение памяти для буфера

    /*if (wiringPiSetupGpio() == -1) {
        perror("GPIO setup failed");
        return 0;
    }

    int gpio_pin = 19; // GPIO номер для 37 пина на плате
    pinMode(gpio_pin, INPUT); // Настройка пина как вход
    pullUpDnControl(gpio_pin, PUD_DOWN); // Подтяжка к "земле" для стабильности
*/
    //while (1) {
        //RxEth(buffer);
        //Tx(buffer);
        /*if (kbhit()) { // Проверка нажатия клавиши
            char key = getchar();
            if (key == 'p') {
                printf("Transmitting audio...\n");
                audioTxEth_client(buffer); // Захват и передача звука
            }
        } else {
            printf("Receiving audio...\n");
            audioRxEth_client(buffer); // Воспроизведение звука
        }*/
        
        audioRxEth_client(buffer);
        //audioTxEth_PI(buffer);
        
        /*if (digitalRead(gpio_pin) == HIGH){
            audioTxEth_PI(buffer);
        }
        if (digitalRead(gpio_pin) == LOW){
            audioRxEth_PI(buffer);
        }*/
        //Rx(buffer);
        //TxEth(buffer);
        
    //}
    free(buffer);

    return 0;
}