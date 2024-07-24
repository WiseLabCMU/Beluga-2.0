//
// Created by tom on 7/8/24.
//

#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>

#define WDT_MIN_WINDOW 0UL

#ifndef CONFIG_WDT_MAX_WINDOW
#define WDT_MAX_WINDOW 2000UL
#else
#define WDT_MAX_WINDOW CONFIG_WDT_MAX_WINDOW
#endif

#ifdef CONFIG_WATCHDOG
#define WDT_NAME    DEVICE_DT_GET(DT_NODELABEL(wdt))
#define WDT_ENABLED true
#else
#define WDT_NAME    NULL
#define WDT_ENABLED false
#endif

static const struct device *wdt;
static int watchdog_id = -1;
static bool starving_dog = false;

int configure_watchdog_timer(void) {
    int err;
    wdt = WDT_NAME;

    if (!WDT_ENABLED) {
        return 0;
    }

    if (!device_is_ready(wdt)) {
        printk("%s: device is not ready.\n", wdt->name);
        return -1;
    }

    struct wdt_timeout_cfg wdt_config = {.flags = WDT_FLAG_RESET_SOC,
                                         .window.min = WDT_MIN_WINDOW,
                                         .window.max = WDT_MAX_WINDOW,
                                         .callback = NULL};

    watchdog_id = wdt_install_timeout(wdt, &wdt_config);

    if (watchdog_id < 0) {
        printk("Watchdog install error (%d)\n", watchdog_id);
        return -1;
    }

    err = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);

    if (err < 0) {
        printk("Watchdog setup failed (%d)\n", err);
        return -1;
    }

    err = wdt_feed(wdt, watchdog_id);

    if (err < 0) {
        printk("Failed to feed the dog (%d)\n", err);
        return -1;
    }

    return 0;
}

void let_the_dog_starve(void) { starving_dog = true; }

void watchdog_red_rocket(void) {
    if (!WDT_ENABLED) {
        return;
    }

    if (!starving_dog) {
        wdt_feed(wdt, watchdog_id);
    }
}
