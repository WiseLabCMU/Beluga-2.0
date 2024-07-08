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

struct buffer {
    uint8_t *buf;
    size_t len;
};

void init_at_commands(void);

/**
 * @brief Task to receive UART message from zephyr UART queue and parse
 */
void runSerialCommand(void);

#endif