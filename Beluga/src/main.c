/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <dk_buttons_and_leds.h>

#include "ble_app.h"
#include <zephyr/kernel.h>
#include <spi.h>

int main(void) {
    int err;

    printk(
        "Starting Bluetooth Central and Peripheral Heart Rate relay example\n");

    err = dk_leds_init();
    if (err) {
        printk("LEDs init failed (err %d)\n", err);
        return 0;
    }

    err = init_bt_stack();

    if (err) {
        printk("Failed to initialize the Bluetooth App\n");
        return 0;
    }

    for (;;) {
    }
}
