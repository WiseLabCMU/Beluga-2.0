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

#include <beluga_message.h>
#include <serial/comms.h>
#include <serial/comms_uart.h>
#include <stdbool.h>
#include <stdio.h>
#include <watchdog.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

/**
 * Logger for the watchdog timer
 */
LOG_MODULE_REGISTER(watchdog_logger, CONFIG_WATCHDOG_MODULE_LOG_LEVEL);

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

#if IS_ENABLED(CONFIG_SW_WDT_HW_FALLBACK)
#define MIN_WDT_PERIOD CONFIG_SW_WDT_HW_FALLBACK_DELAY
#else
#define MIN_WDT_PERIOD CONFIG_SW_WDT_MIN_DELAY
#endif

struct wdt_timeout_work {
    int channel_id;
    k_tid_t tid;
    struct k_work work;
};

struct wdt_channel {
    int64_t period;
    k_tid_t tid;
    uint64_t timeout;
    bool active;
};

struct hw_wdt {
    const struct device *const wdt;
    unsigned int channel;
};

static struct wdt_timeout_work wdt_work;
static struct wdt_channel wdt_channels[CONFIG_SW_WDT_CHANNELS] = {};
struct hw_wdt wdt = {
    .wdt = DEVICE_DT_GET_OR_NULL(WDT_NODE),
};

static struct k_spinlock spinlock;

#define WDT_CHECK(attr_) (attr_) == NULL || attr->id < 0 || attr->id >= ARRAY_SIZE(wdt_channels)

static void report_fatal_error(struct k_work *work) {
    struct wdt_timeout_work *wdt_to_data =
        CONTAINER_OF(work, struct wdt_timeout_work, work);
    char msg_[128];
    struct beluga_msg msg = {
        .type = LOG_FATAL_ERROR,
        .payload.error_message = msg_,
    };
    const struct comms *comms_ptr = comms_backend_uart_get_ptr();

    snprintk(msg_, sizeof(msg), "Task watchdog expired (%s)",
             k_thread_name_get(wdt_to_data->tid));
    comms_write_msg(comms_ptr, &msg);
    comms_flush_out(comms_ptr, 1);

    sys_reboot(SYS_REBOOT_COLD);
}

static void wdt_timer_handler(struct k_timer *timer) {
    int64_t current_ticks = sys_clock_tick_get();
    wdt_feed(wdt.wdt, 0); // todo

    for (ssize_t i = 0; i < ARRAY_SIZE(wdt_channels); i++) {
        if (wdt_channels[i].active && wdt_channels[i].timeout < current_ticks) { // todo
            k_work_submit(&wdt_work.work);
            k_timer_stop(timer);
            break;
        }
    }
}
K_TIMER_DEFINE(wdt_timer, wdt_timer_handler, NULL);

int configure_watchdog_timer(void) {
    if (!device_is_ready(wdt.wdt)) {
        // TODO
    }

    k_work_init(&wdt_work.work, report_fatal_error);

    k_timer_init(&wdt_timer, wdt_timer_handler, NULL);
    k_timer_start(&wdt_timer, K_MSEC(MIN_WDT_PERIOD / 2), K_MSEC(MIN_WDT_PERIOD / 2));

    return 0;
}

int spawn_task_watchdog(struct task_wdt_attr *attr) {
    int ret = -ENOMEM;

    if (attr == NULL || (attr->id >= 0 && attr->id < ARRAY_SIZE(wdt_channels))) {
        return -EINVAL;
    }

    attr->period = MAX(attr->period, (int32_t)MIN_WDT_PERIOD);

    K_SPINLOCK(&spinlock) {
        for (ssize_t i = 0; i < ARRAY_SIZE(wdt_channels); i++) {
            if (!wdt_channels[i].active) {
                // Found an available channel
                wdt_channels[i].period = attr->period;
                wdt_channels[i].active = true;
                wdt_channels[i].tid = k_current_get();
                // TODO: timeout
                attr->id = i;
                attr->starving = false;
                ret = 0;
                break;
            }
        }
    }

    if (ret == 0) {
        watchdog_red_rocket(attr);
    }

    return ret;
}

void let_the_dog_starve(struct task_wdt_attr *attr) {
    if (WDT_CHECK(attr) ) {
        return;
    }
    attr->starving = true;
}

static ALWAYS_INLINE void crit_sect_onexit(unsigned int *key) {
    __ASSERT(*key, "CRITICAL_SECTION exited with goto, break, or return; use CRITICAL_SECTION_BREAK instead.");
    k_sched_unlock();
}
#define CRITICAL_SECTION_ONEXIT __attribute__((cleanup(crit_sect_onexit)))
#define CRITICAL_SECTION_BREAK continue

#define CRITICAL_SECTION() k_sched_lock(); for (unsigned int __i __attribute__((__cleanup__(crit_sect_onexit))) = 0, __key = irq_lock(); !__i; irq_unlock(__key), __i = 1)

void watchdog_red_rocket(struct task_wdt_attr *attr) {
    int64_t current_ticks;

    if (WDT_CHECK(attr) || unlikely(attr->starving)) {
        return;
    }

    // Don't allow anyone to interrupt this process
    CRITICAL_SECTION() {
        // TODO
    }
}

int kill_task_watchdog(struct task_wdt_attr *attr) {

    if (WDT_CHECK(attr)) {
        return -EINVAL;
    }

    K_SPINLOCK(&spinlock) {
        wdt_channels[attr->id].active = false;
        attr->id = -1;
    }

    return 0;
}
