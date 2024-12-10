//
// Created by tom on 7/8/24.
//

#include <stdbool.h>
#include <stdio.h>
#include <watchdog.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/task_wdt/task_wdt.h>

LOG_MODULE_REGISTER(watchdog_logger, CONFIG_WATCHDOG_MODULE_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_wdt) && IS_ENABLED(CONFIG_WATCHDOG)
#define WDT_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(nordic_nrf_wdt)
#else
#warning "No hardware watchdog timer fallback"
#define WDT_NODE DT_INVALID_NODE
#endif

static void task_wdt_callback(int channel_id, void *user_data) {
    LOG_ERR("Watchdog %d has starved. The offending thread (%s) will be tried "
            "for animal cruelty",
            channel_id, k_thread_name_get((k_tid_t)user_data));

    sys_reboot(SYS_REBOOT_COLD);
}

int configure_watchdog_timer(void) {
    int ret;
    const struct device *const wdt = DEVICE_DT_GET_OR_NULL(WDT_NODE);

    if (!IS_ENABLED(CONFIG_TASK_WDT)) {
        LOG_WRN("WDT disabled");
        return 0;
    }

    if (!device_is_ready(wdt)) {
        LOG_ERR("Hardware watchdog is not ready");
    }

    ret = task_wdt_init(wdt);
    if (ret != 0) {
        LOG_ERR("Task wdt init failure: %d", ret);
    }

    return ret;
}

int spawn_task_watchdog(struct task_wdt_attr *attr) {
    int ret;

    if (!IS_ENABLED(CONFIG_TASK_WDT)) {
        LOG_WRN("WDT disabled");
        return 0;
    }

    if (attr == NULL) {
        LOG_WRN("No attributes detected");
        return -1;
    }

    ret =
        task_wdt_add(attr->period, task_wdt_callback, (void *)k_current_get());
    if (ret < 0) {
        LOG_WRN("Unable to spawn puppy (%d)", ret);
        return -1;
    }

    attr->id = ret;
    attr->starving = false;
    return 0;
}

void let_the_dog_starve(struct task_wdt_attr *attr) {
    if (!IS_ENABLED(CONFIG_TASK_WDT)) {
        LOG_WRN("WDT disabled");
        return;
    }

    if (attr == NULL) {
        LOG_WRN("No attributes detected");
        return;
    }
    attr->starving = true;
}

void watchdog_red_rocket(struct task_wdt_attr *attr) {
    if (!IS_ENABLED(CONFIG_TASK_WDT)) {
        LOG_WRN("WDT disabled");
        return;
    }

    if (attr == NULL) {
        LOG_WRN("A non-existent dog cannot get a red rocket");
        return;
    }
    if (!attr->starving) {
        task_wdt_feed(attr->id);
    }
}

int kill_task_watchdog(struct task_wdt_attr *attr) {
    if (!IS_ENABLED(CONFIG_TASK_WDT)) {
        LOG_WRN("WDT disabled");
        return 0;
    }

    if (attr == NULL) {
        LOG_WRN("A non-existent dog cannot be euthanized");
        return 1;
    }

    return task_wdt_delete(attr->id);
}
