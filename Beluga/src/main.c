/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "ble_app.h"
#include <app_leds.h>
#include <at_commands.h>
#include <init_main.h>
#include <led_config.h>
#include <list_monitor.h>
#include <list_neighbors.h>
#include <ranging.h>
#include <resp_main.h>
#include <responder.h>
#include <settings.h>
#include <spi.h>
#include <stdio.h>
#include <timestamp.h>
#include <uart.h>
#include <unistd.h>
#include <watchdog.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>

/* Firmware version */
#define FIRMWARE_VERSION "2.0"

static void load_led_mode(void) {
    int32_t led_mode = retrieveSetting(BELUGA_LEDMODE);

    if (led_mode == 1) {
        all_leds_off();
        printf("  LED Mode: Off \r\n");
    } else if (led_mode == 0) {
        printf("  LED Mode: On \r\n");
    } else {
        printf("  LED Mode: Default \r\n");
    }
    update_led_state(LED_POWER_ON);
}

static void load_id(void) {
    int32_t nodeID = retrieveSetting(BELUGA_ID);

    if (nodeID != DEFAULT_ID_SETTING) {
        update_node_id((uint16_t)nodeID);
        printf("  Node ID: %d \r\n", nodeID);
    } else {
        printf("  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
        printf("  !Warning! Please setup node ID!\r\n");
        printf("  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
    }
}

static void load_bootmode(void) {
    int32_t bootMode = retrieveSetting(BELUGA_BOOTMODE);

    switch (bootMode) {
    case 1:
        k_sem_give(&print_list_sem);
        enable_bluetooth();
        update_led_state(LED_BLE_ON);
        break;
    case 2:
        k_sem_give(&print_list_sem);
        enable_bluetooth();
        update_led_state(LED_BLE_ON);
        k_sem_give(&k_sus_resp);
        k_sem_give(&k_sus_init);
        update_led_state(LED_UWB_ON);
    case 0:
        break;
    default:
        printf("  Boot Mode: Default \r\n");
        break;
    }

    if (bootMode == 0 || bootMode == 1 || bootMode == 2) {
        printf("  Boot Mode: BLE %s + UWB %s\r\n",
               (bootMode > 0) ? "ON" : "OFF", (bootMode == 2) ? "ON" : "OFF");
    }
}

static void load_poll_rate(void) {
    int32_t rate = retrieveSetting(BELUGA_POLL_RATE);

    if (rate != DEFAULT_SETTING) {
        set_rate(rate);
        advertising_reconfig(rate != 0);
        printf("  UWB Polling Rate: %d\r\n", rate);
    } else {
        printf("  UWB Polling Rate: Default \r\n");
    }
}

static void load_channel(void) {
    int32_t channel = retrieveSetting(BELUGA_UWB_CHANNEL);

    if (channel != DEFAULT_SETTING) {
        set_uwb_channel(channel);
        printf("  UWB Channel: %d \r\n", channel);
    } else {
        printf("  UWB Channel: Defalut \r\n");
    }
}

static void load_timeout(void) {
    int32_t timeout = retrieveSetting(BELUGA_BLE_TIMEOUT);

    if (timeout != DEFAULT_SETTING) {
        set_node_timeout(timeout);
        printf("  BLE Timeout: %d \r\n", timeout);
    } else {
        printf("  BLE Timeout: Default \r\n");
    }
}

static void load_tx_power(void) {
    int32_t tx_power = retrieveSetting(BELUGA_TX_POWER);

    if (tx_power == 1) {
        set_tx_power(true);
        printf("  TX Power: Max \r\n");
    } else {
        printf("  TX Power: Default \r\n");
    }
}

static void load_stream_mode(void) {
    int32_t stream_mode = retrieveSetting(BELUGA_STREAMMODE);

    if (stream_mode != DEFAULT_SETTING) {
        set_stream_mode(stream_mode != 0);
        printf("  Stream Mode: %d \r\n", stream_mode);
    } else {
        printf("  Stream Mode: Default \r\n");
    }
}

static void load_twr_mode(void) {
    int32_t twr = retrieveSetting(BELUGA_TWR);

    if (twr != DEFAULT_SETTING) {
        set_twr_mode(twr != 0);
        printf("  Ranging Mode: %d \r\n", twr);
    } else {
        printf("  Ranging Mode: Default \r\n");
    }
}

static void load_settings(void) {
    printf("Flash Configuration: \r\n");

    watchdog_red_rocket();
    load_led_mode();
    watchdog_red_rocket();
    load_id();
    watchdog_red_rocket();
    load_bootmode();
    watchdog_red_rocket();
    load_poll_rate();
    watchdog_red_rocket();
    load_channel();
    watchdog_red_rocket();
    load_timeout();
    watchdog_red_rocket();
    load_tx_power();
    watchdog_red_rocket();
    load_stream_mode();
    watchdog_red_rocket();
    load_twr_mode();
    watchdog_red_rocket();
}

static void get_reset_cause(void) {
    uint32_t reason;
    hwinfo_get_reset_cause(&reason);
    hwinfo_clear_reset_cause();

    printk("Reset causes: \n");

    if (reason & RESET_PIN) {
        printk("External pin\n");
    }
    if (reason & RESET_SOFTWARE) {
        printk("Software reset\n");
    }
    if (reason & RESET_BROWNOUT) {
        printk("Brownout\n");
    }
    if (reason & RESET_POR) {
        printk("Power-on reset\n");
    }
    if (reason & RESET_WATCHDOG) {
        printk("Watchdog timer expiration\n");
    }
    if (reason & RESET_DEBUG) {
        printk("Debug event\n");
    }
    if (reason & RESET_SECURITY) {
        printk("Security violation\n");
    }
    if (reason & RESET_LOW_POWER_WAKE) {
        printk("Waking up from low power mode\n");
    }
    if (reason & RESET_CPU_LOCKUP) {
        printk("CPU lock-up detected\n");
    }
    if (reason & RESET_PARITY) {
        printk("Parity error\n");
    }
    if (reason & RESET_PLL) {
        printk("PLL error\n");
    }
    if (reason & RESET_CLOCK) {
        printk("Clock error\n");
    }
    if (reason & RESET_HARDWARE) {
        printk("Hardware reset\n");
    }
    if (reason & RESET_USER) {
        printk("User reset\n");
    }
    if (reason & RESET_TEMPERATURE) {
        printk("Temperature reset\n");
    }
}

int main(void) {
    get_reset_cause();

    memset(seen_list, 0, ARRAY_SIZE(seen_list));

    if (init_bt_stack() != 0) {
        printk("Failed to init bluetooth stack\n");
    }

    if (initBelugaSettings()) {
        printk("Unable to init flash\n");
        return 1;
    }

    if (uart_init() < 0) {
        printk("Failed to init uart\n");
        return 1;
    }

    if (init_spi1() < 0) {
        printk("Failed to initialize SPI 1\n");
        return 1;
    }

    LED_INIT;

    if (configure_watchdog_timer() < 0) {
        printk("Failed to configure watchdog timer\n");
        return 1;
    }

    init_uwb();

    init_timekeeper(1, 1);

    printf("Node On: Firmware version %s\r\n", FIRMWARE_VERSION);

    load_settings();
    printf("\r\n");

    init_responder_thread();
    init_commands_thread();
    init_print_list_task();
    init_ranging_thread();
    init_monitor_thread();

    for (;;) {
        k_sleep(K_FOREVER);
    }

    return 1;
}
