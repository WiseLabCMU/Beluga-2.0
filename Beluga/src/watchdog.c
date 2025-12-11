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
    const char *name;
    struct k_work work;
};

struct wdt_channel {
    int64_t period;
    k_tid_t tid;
    const char *name;
    int64_t timeout_ms;
    bool active;
};

struct hw_wdt {
    const struct device *const wdt;
    int channel;
    atomic_t state;
};

enum {
    HW_WDT_STARTED,
};

static struct wdt_timeout_work wdt_work;
static struct wdt_channel wdt_channels[CONFIG_SW_WDT_CHANNELS] = {};
static struct hw_wdt wdt = {
    .wdt = DEVICE_DT_GET_OR_NULL(WDT_NODE),
};

static struct k_spinlock spinlock;

#define WDT_CHECK(attr_)                                                       \
    (attr_) == NULL || attr->id < 0 || attr->id >= ARRAY_SIZE(wdt_channels)

static void report_fatal_error(struct k_work *work) {
    struct wdt_timeout_work *wdt_to_data =
        CONTAINER_OF(work, struct wdt_timeout_work, work);
    char msg_[128];
    struct beluga_msg msg = {
        .type = LOG_FATAL_ERROR,
        .payload.error_message = msg_,
    };
    const struct comms *comms_ptr = comms_backend_uart_get_ptr();
    const char *thread_name = wdt_to_data->name;

    snprintf(msg_, sizeof(msg_), "Task watchdog expired (%s)", thread_name);
    comms_write_msg(comms_ptr, &msg);
    // fatal errors are written in a blocking enabled.
    k_sleep(K_MSEC(MIN_WDT_PERIOD / 2));

    sys_reboot(SYS_REBOOT_COLD);
}

static void wdt_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(wdt_timer, wdt_timer_handler, NULL);

static void schedule_next_timeout(int64_t current_time_ms) {
    size_t next_channel_id;
    int64_t next_timeout;

#if defined(CONFIG_SW_WDT_HW_FALLBACK)
    next_channel_id = ARRAY_SIZE(wdt_channels);
    next_timeout =
        current_time_ms + (int64_t)(CONFIG_SW_WDT_HW_FALLBACK_DELAY / 2);
#else
    next_channel_id = 0;
    next_timeout = INT64_MAX;
#endif // defined(CONFIG_SW_WDT_HW_FALLBACK)

    for (size_t i = 0; i < ARRAY_SIZE(wdt_channels); i++) {
        if (wdt_channels[i].active &&
            wdt_channels[i].timeout_ms < next_timeout) {
            next_channel_id = i;
            next_timeout = wdt_channels[i].timeout_ms;
        }
    }

    k_timer_user_data_set(&wdt_timer, (void *)next_channel_id);
    k_timer_start(&wdt_timer, K_MSEC(next_timeout - current_time_ms),
                  K_FOREVER);

#if defined(CONFIG_SW_WDT_HW_FALLBACK)
    if (atomic_test_bit(&wdt.state, HW_WDT_STARTED)) {
        wdt_feed(wdt.wdt, wdt.channel);
    }
#endif // defined(CONFIG_SW_WDT_HW_FALLBACK)
}

static void wdt_timer_handler(struct k_timer *timer) {
    size_t channel_id = (size_t)k_timer_user_data_get(timer);
    bool bg_channel = IS_ENABLED(CONFIG_SW_WDT_HW_FALLBACK) &&
                      (channel_id == ARRAY_SIZE(wdt_channels));

    if (bg_channel || !wdt_channels[channel_id].active) {
        schedule_next_timeout(k_uptime_get());
        return;
    }

#if defined(CONFIG_SW_WDT_HW_FALLBACK)
    if (atomic_test_bit(&wdt.state, HW_WDT_STARTED)) {
        wdt_feed(wdt.wdt, wdt.channel);
    }
#endif // defined(CONFIG_SW_WDT_HW_FALLBACK)
    wdt_work.name = wdt_channels[channel_id].name;
    k_work_submit(&wdt_work.work);
}

int configure_watchdog_timer(void) {
#if defined(CONFIG_SW_WDT_HW_FALLBACK)
    struct wdt_timeout_cfg wdt_config;
    if (!device_is_ready(wdt.wdt)) {
        return -ENODEV;
    }

    wdt_config.flags = WDT_FLAG_RESET_SOC;
    wdt_config.window.min = 0U;
    wdt_config.window.max = CONFIG_SW_WDT_HW_FALLBACK_DELAY;
    wdt_config.callback = NULL;

    wdt.channel = wdt_install_timeout(wdt.wdt, &wdt_config);
    if (wdt.channel < 0) {
        LOG_ERR("Unable to install hw watchdog timeout: %d", wdt.channel);
        return wdt.channel;
    }
#endif // defined(CONFIG_SW_WDT_HW_FALLBACK)

    k_work_init(&wdt_work.work, report_fatal_error);
    schedule_next_timeout(k_uptime_get());

    return 0;
}

int spawn_task_watchdog(struct task_wdt_attr *attr) {
    int ret = -ENOMEM;

    if (attr == NULL ||
        (attr->id >= 0 && attr->id < ARRAY_SIZE(wdt_channels))) {
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
                wdt_channels[i].timeout_ms = K_TICKS_FOREVER;
                wdt_channels[i].name = attr->name;
                attr->id = i;
                attr->starving = false;
                ret = 0;

#if defined(CONFIG_SW_WDT_HW_FALLBACK)
                if (!atomic_test_bit(&wdt.state, HW_WDT_STARTED) && wdt.wdt) {
                    wdt_setup(wdt.wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
                    atomic_set_bit(&wdt.state, HW_WDT_STARTED);
                }
#endif // defined(CONFIG_SW_WDT_HW_FALLBACK)
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
    if (WDT_CHECK(attr)) {
        return;
    }
    attr->starving = true;
}

static ALWAYS_INLINE void crit_sect_onexit(unsigned int *key) {
    __ASSERT(*key, "CRITICAL_SECTION exited with goto, break, or return; use "
                   "CRITICAL_SECTION_BREAK instead.");
    k_sched_unlock();
}
#define CRITICAL_SECTION_ONEXIT __attribute__((cleanup(crit_sect_onexit)))
#define CRITICAL_SECTION_BREAK  continue

#define CRITICAL_SECTION()                                                     \
    k_sched_lock();                                                            \
    for (unsigned int __i __attribute__((__cleanup__(crit_sect_onexit))) = 0,  \
                          __key = irq_lock();                                  \
         !__i; irq_unlock(__key), __i = 1)

void watchdog_red_rocket(struct task_wdt_attr *attr) {
    int64_t current_uptime;

    if (WDT_CHECK(attr) || unlikely(attr->starving)) {
        return;
    }

    // Don't allow anyone to interrupt this process
    CRITICAL_SECTION() {
        current_uptime = k_uptime_get();

        wdt_channels[attr->id].timeout_ms =
            current_uptime + wdt_channels[attr->id].period;
        schedule_next_timeout(current_uptime);
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

int set_watchdog_tid(const struct task_wdt_attr *attr, k_tid_t tid) {
    if (WDT_CHECK(attr)) {
        return -EINVAL;
    }

    K_SPINLOCK(&spinlock) { wdt_channels[attr->id].tid = tid; }

    return 0;
}
