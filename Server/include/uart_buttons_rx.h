#pragma once

// слушает TCP (port), принимает команды по 5 байт и пишет их в UART (tty, baud)
// RTS/CTS can be mapped to GPIO lines (for direct UART0 wiring without USB-UART modem lines).
void uart_buttons_rx_thread(int port,
                            const char* tty,
                            int baud_bps,
                            const char* gpiochip_path,
                            unsigned rts_line_offset,
                            unsigned cts_line_offset,
                            bool rts_active_low,
                            bool cts_active_low,
                            bool tx_invert);
void uart_buttons_rx_stop();
