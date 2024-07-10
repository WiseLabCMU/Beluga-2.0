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
    void *fifo_reserved;
    uint8_t *buf;
    size_t len;
};

#endif