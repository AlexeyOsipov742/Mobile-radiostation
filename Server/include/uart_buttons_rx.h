#pragma once

// слушает TCP (port), принимает команды по 5 байт и пишет их в UART (tty, baud)
void uart_buttons_rx_thread(int port, const char* tty, int baud_bps);
void uart_buttons_rx_stop();
