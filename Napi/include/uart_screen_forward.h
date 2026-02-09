#pragma once

// Старт фонового форвардера SB9600: UART (RAW) -> TCP (len+payload)
//
// pi_ip     — IP пульта (Pi)
// port      — TCP порт на пульте (например 7777)
// tty       — например "/dev/ttyUSB0"
// baud_bps  — пока игнорируется (фиксируем 9600 как в SB9600)
// window_ms — "окно" чтения UART, как в debug_server
void uart_screen_forward_thread(const char* pi_ip, int port, const char* tty, int baud_bps, int window_ms);

// Опционально: мягкая остановка (если потом добавим обработку сигналов).
void uart_screen_forward_stop();

