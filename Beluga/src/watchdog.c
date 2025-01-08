/**
 * @file watchdog.c
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

#include <stdbool.h>
#include <stdio.h>
#include <watchdog.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/task_wdt/task_wdt.h>

/**
 * Logger for the watchdog timer
 */
LOG_MODULE_REGISTER(watchdog_logger, CONFIG_WATCHDOG_MODULE_LOG_LEVEL);

#if defined(CONFIG_TASK_WDT)

#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_wdt) && IS_ENABLED(CONFIG_WATCHDOG)
/**
 * The hardware watchdog timer node from the devicetree
 */
#define WDT_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(nordic_nrf_wdt)
#else
#warning "No hardware watchdog timer fallback"

/**
 * The hardware watchdog timer node from the devicetree
 */
#define WDT_NODE DT_INVALID_NODE
#endif

/**
 * @brief Callback that gets called whenever a watchdog channel expires
 * @param[in] channel_id The task watchdog channel ID
 * @param[in] user_data Additional context
 */
static void task_wdt_callback(int channel_id, void *user_data) {
    ARG_UNUSED(channel_id);
    ARG_UNUSED(user_data);
    LOG_ERR("Watchdog %d has starved. The offending thread (%s) will be tried "
            "for animal cruelty",
            channel_id, k_thread_name_get((k_tid_t)user_data));

    sys_reboot(SYS_REBOOT_COLD);
}

/**
 * @brief Initializes the watchdog timer and the task watchdog subsystem
 * @return 0 upon success
 * @return negative error code otherwise
 */
int configure_watchdog_timer(void) {
    int ret;
    const struct device *const wdt = DEVICE_DT_GET_OR_NULL(WDT_NODE);

    if (!device_is_ready(wdt)) {
        LOG_ERR("Hardware watchdog is not ready");
    }

    ret = task_wdt_init(wdt);
    if (ret != 0) {
        LOG_ERR("Task wdt init failure: %d", ret);
    }

    return ret;
}

/**
 * @brief Creates a new task watchdog timer and initializes it
 * @param[in,out] attr The task watchdog attributes to initialize
 * @return 0 upon success
 * @return -EINVAL if attr is NULL
 * @return negative error code otherwise
 */
int spawn_task_watchdog(struct task_wdt_attr *attr) {
    int ret;

    if (attr == NULL) {
        LOG_WRN("No attributes detected");
        return -EINVAL;
    }

    ret =
        task_wdt_add(attr->period, task_wdt_callback, (void *)k_current_get());
    if (ret < 0) {
        LOG_WRN("Unable to spawn puppy (%d)", ret);
        return ret;
    }

    attr->id = ret;
    attr->starving = false;
    return 0;
}

/**
 * @brief Sets an attribute that prevents the task watchdog timer from feeding
 * @param[in] attr The task watchdog attributes
 */
void let_the_dog_starve(struct task_wdt_attr *attr) {
    if (attr == NULL) {
        LOG_WRN("No attributes detected");
        return;
    }
    attr->starving = true;
}

/**
 * @brief Feeds a task watchdog timer if it is not marked for starving
 *
 * Come here Sparky
 * Red rocket, Red rocket! heh-heh-heh Red Rocket, Red Rocket! heh-heh Come on!
 * Red rocket, come on, dog, red rocket. Oh, hoo!
 *
 * @param[in] attr The task watchdog attributes
 */
void watchdog_red_rocket(struct task_wdt_attr *attr) {
    if (attr == NULL) {
        LOG_WRN("A non-existent dog cannot get a red rocket");
        return;
    }
    if (!attr->starving) {
        task_wdt_feed(attr->id);
    }
}

/**
 * @brief Deletes a task watchdog channel
 * @param[in] attr The task watchdog attribute for the dog to be euthanized
 * @return 0 upon success
 * @return -EINVAL if attr is NULL
 * @return negative error code otherwise
 */
int kill_task_watchdog(struct task_wdt_attr *attr) {
    if (attr == NULL) {
        LOG_WRN("A non-existent dog cannot be euthanized");
        return -EINVAL;
    }

    return task_wdt_delete(attr->id);
}

#else

/**
 * Replacement code for watchdog functions that must return
 */
#define _DISABLED_RET(ret)                                                     \
    do {                                                                       \
        LOG_WRN("WDT disabled");                                               \
        return ret;                                                            \
    } while (0)

/**
 * Generator for logging a watchdog warning and returning
 */
#define WDT_DISABLED(ret...)                                                   \
    COND_CODE_1(IS_EMPTY(ret), (LOG_WRN("WDT disabled")),                      \
                (_DISABLED_RET(GET_ARG_N(1, ret))))

/**
 * @brief Initializes the watchdog timer and the task watchdog subsystem
 * @return 0 upon success
 * @return negative error code otherwise
 */
int configure_watchdog_timer(void) { WDT_DISABLED(0); }

/**
 * @brief Creates a new task watchdog timer and initializes it
 * @param[in,out] attr The task watchdog attributes to initialize
 * @return 0 upon success
 * @return -EINVAL if attr is NULL
 * @return negative error code otherwise
 */
int spawn_task_watchdog(struct task_wdt_attr *attr) {
    ARG_UNUSED(attr);
    WDT_DISABLED(0);
}

/**
 * @brief Sets an attribute that prevents the task watchdog timer from feeding
 * @param[in] attr The task watchdog attributes
 */
void let_the_dog_starve(struct task_wdt_attr *attr) {
    ARG_UNUSED(attr);
    WDT_DISABLED();
}

/**
 * @brief Feeds a task watchdog timer if it is not marked for starving
 *
 * Come here Sparky
 * Red rocket, Red rocket! heh-heh-heh Red Rocket, Red Rocket! heh-heh Come on!
 * Red rocket, come on, dog, red rocket. Oh, hoo!
 *
 * @param[in] attr The task watchdog attributes
 */
void watchdog_red_rocket(struct task_wdt_attr *attr) {
    ARG_UNUSED(attr);
    WDT_DISABLED();
}

/**
 * @brief Deletes a task watchdog channel
 * @param[in] attr The task watchdog attribute for the dog to be euthanized
 * @return 0 upon success
 * @return -EINVAL if attr is NULL
 * @return negative error code otherwise
 */
int kill_task_watchdog(struct task_wdt_attr *attr) {
    ARG_UNUSED(attr);
    WDT_DISABLED(0);
}

#endif // defined(CONFIG_TASK_WDT)
