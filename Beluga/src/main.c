/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "ble_app.h"
#include <led_config.h>
#include <spi.h>
#include <zephyr/kernel.h>
#include <port_platform.h>
#include <watchdog.h>

int main(void) {
    int err;

    printk(
        "Starting Bluetooth Central and Peripheral Heart Rate relay example\n");

    LED_INIT;

    err = init_bt_stack();

    if (err) {
        printk("Failed to initialize the Bluetooth App\n");
        return 0;
    }

    if (configure_watchdog_timer() < 0) {
        printk("Failed to start watchdog\n");
        return 0;
    }

    for (;;) {
    }
}
