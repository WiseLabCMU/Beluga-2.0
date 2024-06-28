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

int main(void) {
    int err;

    printk(
        "Starting Bluetooth Central and Peripheral Heart Rate relay example\n");

    LED_INIT;

    if (init_spi1()) {
        printk("Failure");
        return 1;
    }

    uint8_t buf[] = "Hello World!";

    if (write_spi(DW1000_SPI_CHANNEL, buf, sizeof(buf) - 1)) {
        printk("Transfer error");
        return 1;
    }

    err = init_bt_stack();

    if (err) {
        printk("Failed to initialize the Bluetooth App\n");
        return 0;
    }

    for (;;) {
    }
}
