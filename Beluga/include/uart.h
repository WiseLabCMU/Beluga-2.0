/**
 * @file uart.c
 * @brief Serial communication driver for handling UART data reception and
 * transmission.
 *
 * This file contains the implementation for initializing and managing UART
 * communication. It includes:
 * - UART interrupt handling
 * - Buffer management for received UART data
 * - Serial output hooks for custom console output behavior
 * - LED control integration during UART activity
 *
 * @date 7/8/2024
 * @author Tom Schmitz
 */

#ifndef BELUGA_UART_H
#define BELUGA_UART_H

#include <zephyr/kernel.h>

/**
 * @brief Initializes the serial connection by enabling interrupts, waiting for
 * the peer to be ready, and installing the output hooks.
 * @return 0 upon success
 * @return -ENODEV if serial device is not ready
 * @return negative error code otherwise
 */
int uart_init(void);

/**
 * Received data over serial
 */
extern struct k_fifo uart_rx_queue;

#endif // BELUGA_UART_H
