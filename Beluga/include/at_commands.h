/** @file   at_commands.h
 *
 *  @brief  Beluga AT commands
 *
 * This defines all the AT commands available on Beluga. It not only defines
 * the AT commands, but also parses the arguments, provides argument checking
 * helpers, and runs the commands thread.
 *
 *  @date   6/1/2024
 *
 *  @author Tom Schmitz
 */

#ifndef AT_COMMANDS_H
#define AT_COMMANDS_H

#include <zephyr/kernel.h>

#include <stdint.h>
#include <unistd.h>

/**
 * Buffer structure for passing serial messages from the UART to the commands
 * task through a FIFO
 */
struct buffer {
    void *fifo_reserved; ///< Reserved for Zephyr FIFO use
    uint8_t *buf;        ///< The data received from the UART
    size_t len;          ///< The length of the data
};

/**
 * Initializes and launches the commands task and gives the thread a name.
 */
void init_commands_thread(void);

#endif