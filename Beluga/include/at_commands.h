/** @file   at_commands.h
 *
 *  @brief  Beluga AT commands
 *
 *  @date   2024/06
 *
 *  @author WiseLab-CMU
 */

#ifndef AT_COMMANDS_H
#define AT_COMMANDS_H

#include <zephyr/kernel.h>

#include <stdint.h>
#include <unistd.h>

struct command_buffer {
    char *command;
    size_t len;
};

void init_at_commands(void);

/**
 * @brief Task to receive UART message from freertos UART queue and parse
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the
 * task.
 */
void uart_task_function(void *pvParameter);

extern struct k_fifo uart_queue;

#endif