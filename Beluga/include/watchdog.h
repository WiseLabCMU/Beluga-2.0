/**
 * @file watchdog.h
 * @brief Implementation of the watchdog timer functionality.
 *
 * It includes functionality for configuring the hardware watchdog, setting up
 * task watchdog timers, feeding or letting them starve, and handling task
 * expiration events through a defined callback function.
 *
 * The code also provides fallback behavior when the task watchdog feature is
 * not enabled. In the case where the task watchdog subsystem is disabled,
 * functions return early with appropriate warnings.
 *
 * @date 7/8/24
 * @author Tom Schmitz
 */

#ifndef BELUGA_WATCHDOG_H
#define BELUGA_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @struct task_wdt_attr
 * @brief Task watchdog attributes structure
 *
 * Holds the attributes for a task watchdog timer, including the channel ID,
 * a flag indicating whether the task is "starving" (not to be fed), and the
 * period (in milliseconds) before the task watchdog will expire.
 *
 * The `task_wdt_attr` structure is used to configure and manage task watchdog
 * timers for specific threads or tasks. A watchdog channel is identified by a
 * unique ID, and each channel has a configured period after which the task will
 * be considered expired if not fed.
 */
struct task_wdt_attr {
    int32_t id;      ///< Task watchdog channel ID
    bool starving;   ///< Parameter indicating channel is not supposed to be fed
    uint32_t period; ///< The number of milliseconds before the task watchdog
                     ///< starves
};

/**
 * @brief Initializes the watchdog timer and the task watchdog subsystem
 * @return 0 upon success
 * @return negative error code otherwise
 */
int configure_watchdog_timer(void);

/**
 * @brief Creates a new task watchdog timer and initializes it
 * @param[in,out] attr The task watchdog attributes to initialize
 * @return 0 upon success
 * @return -EINVAL if attr is NULL
 * @return negative error code otherwise
 */
int spawn_task_watchdog(struct task_wdt_attr *attr);

/**
 * @brief Sets an attribute that prevents the task watchdog timer from feeding
 * @param[in] attr The task watchdog attributes
 */
void let_the_dog_starve(struct task_wdt_attr *attr);

/**
 * @brief Feeds a task watchdog timer if it is not marked for starving
 *
 * Come here Sparky
 * Red rocket, Red rocket! heh-heh-heh Red Rocket, Red Rocket! heh-heh Come on!
 * Red rocket, come on, dog, red rocket. Oh, hoo!
 *
 * @param[in] attr The task watchdog attributes
 */
void watchdog_red_rocket(struct task_wdt_attr *attr);

/**
 * @brief Deletes a task watchdog channel
 * @param[in] attr The task watchdog attribute for the dog to be euthanized
 * @return 0 upon success
 * @return -EINVAL if attr is NULL
 * @return negative error code otherwise
 */
int kill_task_watchdog(struct task_wdt_attr *attr);

#endif // BELUGA_WATCHDOG_H
