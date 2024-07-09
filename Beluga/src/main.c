/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "ble_app.h"
#include <led_config.h>
#include <port_platform.h>
#include <spi.h>
#include <watchdog.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <uart.h>
#include <unistd.h>

int main(void) {
    int err;

    LED_INIT;

    if (uart_init() < 0) {
        printf("Error\n");
        return 0;
    }

    printf("Success!\n");
    printf(
       "Starting Bluetooth Central and Peripheral Heart Rate relay example\n");

    err = init_bt_stack();

    if (err) {
        printk("Failed to initialize the Bluetooth App\n");
        return 0;
    }

    // if (configure_watchdog_timer() < 0) {
    //     printk("Failed to start watchdog\n");
    //     return 0;
    // }

    for (;;) {
        k_sleep(K_FOREVER);
    }
}
