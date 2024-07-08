//
// Created by tom on 7/8/24.
//

#ifndef BELUGA_UART_H
#define BELUGA_UART_H

#include <zephyr/kernel.h>

int uart_init(void);

extern struct k_fifo uart_rx_queue;

#endif // BELUGA_UART_H
