/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "ble_app.h"
#include <app_leds.h>
#include <flash.h>
#include <led_config.h>
#include <ranging.h>
#include <spi.h>
#include <stdio.h>
#include <timestamp.h>
#include <uart.h>
#include <unistd.h>
#include <watchdog.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <settings.h>

/* Firmware version */
#define FIRMWARE_VERSION "2.0"

static void load_settings(void) {
    printf("Flash Configuration: \r\n");

    int32_t led_mode = retrieveSetting(BELUGA_LEDMODE);

    if (led_mode == 1) {
        all_leds_off();
        printf("  LED Mode: Off \r\n");
    } else if (led_mode == 0) {
        update_led_state(LED_POWER_ON);
        printf("  LED Mode: On \r\n");
    } else {
        update_led_state(LED_POWER_ON);
        printf("  LED Mode: Default \r\n");
    }
}

int main(void) {
    // if (configure_watchdog_timer() < 0) {
    //     printk("Failed to configure watchdog timer\n");
    //     return 0;
    // }

    uint32_t reason;

    hwinfo_get_reset_cause(&reason);
    hwinfo_clear_reset_cause();

    memset(seen_list, 0, ARRAY_SIZE(seen_list));

    enable_bluetooth();
    if (initBelugaSettings()) {
        printk("Unable to init flash\n");
        return 0;
    }
    // eraseRecords();

    if (uart_init() < 0) {
        printk("Failed to init uart\n");
        return 0;
    }

    LED_INIT;

    // init_uwb();

    // init_timekeeper(1, 1);

    printf("Node On: Firmware version %s\r\n", FIRMWARE_VERSION);

    // TODO: STack sizes

    load_settings();

    for (;;) {
        k_sleep(K_FOREVER);
    }
}
