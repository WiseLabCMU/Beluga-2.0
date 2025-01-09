/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "ble_app.h"
#include <app_leds.h>
#include <app_version.h>
#include <at_commands.h>
#include <debug.h>
#include <initiator.h>
#include <led_config.h>
#include <list_monitor.h>
#include <list_neighbors.h>
#include <range_extension.h>
#include <ranging.h>
#include <responder.h>
#include <settings.h>
#include <spi.h>
#include <stdio.h>
#include <uart.h>
#include <unistd.h>
#include <utils.h>
#include <voltage_regulator.h>
#include <watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main_app, CONFIG_BELUGA_MAIN_LOG_LEVEL);

static void load_led_mode(void) {
    int32_t led_mode = retrieveSetting(BELUGA_LEDMODE);

    if (led_mode == 1) {
        all_leds_off();
    }
    update_led_state(LED_POWER_ON);
    printf("  LED Mode: %d \r\n", led_mode);
}

static void load_id(void) {
    int32_t nodeID = retrieveSetting(BELUGA_ID);

    if (nodeID != DEFAULT_ID_SETTING) {
        update_node_id((uint16_t)nodeID);
        set_initiator_id((uint16_t)nodeID);
        set_responder_id((uint16_t)nodeID);
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
        printf("  Boot Mode: Unknown \r\n");
        break;
    }
    printf("  Boot Mode: %d\r\n", bootMode);
}

static void load_poll_rate(void) {
    int32_t rate = retrieveSetting(BELUGA_POLL_RATE);
    set_rate(rate);
    advertising_reconfig(rate != 0);
    printf("  UWB Polling Rate: %d\r\n", rate);
}

static void load_channel(void) {
    int32_t channel = retrieveSetting(BELUGA_UWB_CHANNEL);
    set_uwb_channel(channel);
    printf("  UWB Channel: %d \r\n", channel);
}

static void load_timeout(void) {
    int32_t timeout = retrieveSetting(BELUGA_BLE_TIMEOUT);
    set_node_timeout(timeout);
    printf("  BLE Timeout: %d \r\n", timeout);
}

static void load_tx_power(void) {
    int32_t tx_power = retrieveSetting(BELUGA_TX_POWER);
    set_tx_power((uint32_t)tx_power);
    printf("  ");
    print_tx_power((uint32_t)tx_power);
    printf("\r\n");
}

static void load_stream_mode(void) {
    int32_t stream_mode = retrieveSetting(BELUGA_STREAMMODE);
    set_stream_mode(stream_mode != 0);
    printf("  Stream Mode: %d \r\n", stream_mode);
}

static void load_twr_mode(void) {
    int32_t twr = retrieveSetting(BELUGA_TWR);
    set_twr_mode(twr != 0);
    printf("  Ranging Mode: %d \r\n", twr);
}

static void load_out_format(void) {
    int32_t format = retrieveSetting(BELUGA_OUT_FORMAT);
    set_format_mode(format == 1);
    printf("  ");
    print_output_format(format);
    printf("\r\n");
}

static void load_phr_mode(void) {
    int32_t mode = retrieveSetting(BELUGA_UWB_PHR);
    printf("  UWB PHR Mode: %" PRId32 " \r\n", mode);
    uwb_set_phr_mode((enum uwb_phr_mode)mode);
}

static void load_data_rate(void) {
    enum uwb_datarate rate =
        (enum uwb_datarate)retrieveSetting(BELUGA_UWB_DATA_RATE);
    printf("  ");
    rate = print_uwb_datarate(rate);
    printf("\r\n");
    uwb_set_datarate(rate);
}

static void load_pulse_rate(void) {
    enum uwb_pulse_rate rate =
        (enum uwb_pulse_rate)retrieveSetting(BELUGA_UWB_PULSE_RATE);
    printf("  ");
    rate = print_pulse_rate(rate);
    printf("\r\n");
    uwb_set_pulse_rate((enum uwb_pulse_rate)rate);
}

static void load_preamble_length(void) {
    int32_t length = retrieveSetting(BELUGA_UWB_PREAMBLE);
    printf("  UWB Preamble Length: %" PRId32 "\r\n", length);
    uwb_set_preamble((enum uwb_preamble_length)length);
}

static void load_pac_size(void) {
    int32_t pac = retrieveSetting(BELUGA_UWB_PAC);
    printf("  ");
    pac = print_pac_size(pac);
    printf("\r\n");
    set_pac_size((enum uwb_pac)pac);
}

static void load_sfd_mode(void) {
    int32_t mode = retrieveSetting(BELUGA_UWB_NSSFD);
    printf("  UWB Nonstandard SFD Length: %" PRId32 " \r\n", mode);
    set_sfd_mode((enum uwb_sfd)mode);
}

static void load_pan_id(void) {
    int32_t pan_id = retrieveSetting(BELUGA_PAN_ID);
    printf("  ");
    print_pan_id(pan_id);
    printf("\r\n");
    set_initiator_pan_id((uint16_t)pan_id);
    set_responder_pan_id((uint16_t)pan_id);
}

UNUSED static void load_power_amplifiers(void) {
    int32_t pwramp = retrieveSetting(BELUGA_RANGE_EXTEND);

    if (pwramp == 1) {
        update_power_mode(POWER_MODE_LOW);
    } else {
        update_power_mode(POWER_MODE_BYPASS);
    }
    printf("  Range Extension: %d \r\n", pwramp);
}

static void load_settings(void) {
    printf("Flash Configuration: \r\n");

    load_led_mode();
    load_id();
    load_bootmode();

    // Disable UWB LED since setters check LED if UWB is active or not
    enum led_state uwb_state = get_uwb_led_state();
    update_led_state(LED_UWB_OFF);

    // Load UWB settings since ranging task has not started yet
    load_poll_rate();
    load_channel();
    load_phr_mode();
    load_data_rate();
    load_pulse_rate();
    load_preamble_length();
    load_pac_size();
    load_sfd_mode();
    load_pan_id();

    // Restore UWB state in the LEDs
    update_led_state(uwb_state);

    load_timeout();
    load_tx_power();
    load_stream_mode();
    load_twr_mode();
    load_out_format();
    if (IS_ENABLED(CONFIG_BELUGA_RANGE_EXTENSION)) {
        load_power_amplifiers();
    }
}

int main(void) {
    RESET_CAUSE();

    memset(seen_list, 0, sizeof(seen_list));

    INIT_CLOCKS();

    if (init_voltage_regulator() != 0) {
        printk("Failed to initialize voltage regulator\n");
        return 1;
    }

    if (init_bt_stack() != 0) {
        printk("Failed to init bluetooth stack\n");
    }

    if (initBelugaSettings() != 0) {
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

    if (init_range_extension() != 0) {
        printk("Failed to initialize range extension\n");
        return 1;
    }

    LED_INIT();

    if (configure_watchdog_timer() < 0) {
        printk("Failed to configure watchdog timer\n");
        return 1;
    }

    struct task_wdt_attr task_watchdog = {.period = 2000};

    if (spawn_task_watchdog(&task_watchdog) != 0) {
        printk("Unable to start watchdog\n");
        return 0;
    }

    init_uwb();

    printf("Node On: " APP_VERSION_STRING "\r\n");

    load_settings();
    printf("\r\n");

    kill_task_watchdog(&task_watchdog);

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
