/**
 * @file debug.h
 *
 * @brief Contains various functions that examine why the code is crashing and
 * logs the crash reasons
 *
 * @date 1/9/25
 * @author Tom Schmitz
 */

#ifndef BELUGA_DEBUG_H
#define BELUGA_DEBUG_H

#include <stdbool.h>
#include <zephyr/kernel.h>

#define HALT_REASON_LF_CLK 0
#define HALT_REASON_HF_CLK 1

#if defined(CONFIG_DEBUG_BELUGA_CLOCK)

/**
 * @brief Defines the external clocks to attempt to initialize
 */
enum clk_cntrl {
    HIGH_FREQ, ///< High frequency clock
    LOW_FREQ   ///< Low frequency clock
};

/**
 * @brief Initializes a clock subsystem and checks if it runs correctly
 * @param[in] clk_subsys The clock subsystem to initialize
 * @return `true` if subsystem initialized correctly
 * @return `false` if the subsystem failed to initialize
 */
bool clock_init(enum clk_cntrl clk_subsys);

/**
 * Initializes the clock subsystem for both the low frequency clock and the high
 * frequency clock. If one fails to initialize, the program
 */
#define INIT_CLOCKS()                                                          \
    do {                                                                       \
        bool retVal = clock_init(LOW_FREQ);                                    \
        if (!retVal) {                                                         \
            k_fatal_halt(HALT_REASON_LF_CLK);                                  \
        }                                                                      \
        retVal = clock_init(HIGH_FREQ);                                        \
        if (!retVal) {                                                         \
            k_fatal_halt(HALT_REASON_HF_CLK);                                  \
        }                                                                      \
    } while (0)
#else

/**
 * Initializes the clock subsystem for both the low frequency clock and the high
 * frequency clock. If one fails to initialize, the program
 */
#define INIT_CLOCKS() (void)0
#endif

#if defined(CONFIG_BELUGA_RESET_REASON)
/**
 * @brief Retrieve the reason why the hardware reset and indicate it to the
 * logger. Additionally, clears the reset reason for the next run.
 */
void get_reset_cause(void);

/**
 * Get the reset cause a log it
 */
#define RESET_CAUSE() get_reset_cause()
#else

/**
 * Get the reset cause a log it
 */
#define RESET_CAUSE() (void)0
#endif

#endif // BELUGA_DEBUG_H
