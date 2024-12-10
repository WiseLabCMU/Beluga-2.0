//
// Created by tom on 7/8/24.
//

#ifndef BELUGA_UART_H
#define BELUGA_UART_H

#include <zephyr/kernel.h>

int uart_init(void);

int __printf_like(1, 2) beluga_printf(const char *ZRESTRICT format, ...);

extern struct k_fifo uart_rx_queue;

#define printf beluga_printf

#endif // BELUGA_UART_H
