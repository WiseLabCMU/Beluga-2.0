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
#include <range_extension.h>
#include <ranging.h>
#include <resp_main.h>
#include <settings.h>
#include <spi.h>
#include <stdio.h>
#include <uart.h>
#include <unistd.h>
#include <utils.h>
#include <voltage_regulator.h>
#include <watchdog.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>

/* Firmware version */
#define FIRMWARE_VERSION "2.0"

enum clk_cntrl { HIGH_FREQ, LOW_FREQ };

UNUSED static bool clock_init(enum clk_cntrl clk_subsys) {
    int err;
    int res;
    struct onoff_manager *clk_mgr;
    struct onoff_client clk_cli;

    switch (clk_subsys) {
    case HIGH_FREQ:
        clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
        break;
    case LOW_FREQ:
        clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_LF);
        break;
    default:
        return false;
    }

    if (!clk_mgr) {
        printk("Unable to get the Clock manager\n");
        return false;
    }

    sys_notify_init_spinwait(&clk_cli.notify);

    err = onoff_request(clk_mgr, &clk_cli);
    if (err < 0) {
        printk("Clock request failed: %d\n", err);
        return false;
    }

    int retries = 15;

    do {
        for (int i = 0; i < 10000; i++)
            ;
        err = sys_notify_fetch_result(&clk_cli.notify, &res);
        if (!err && res) {
            printk("Clock could not be started: %d\n", res);
            return false;
        } else if (err) {
            printk("sys_notify_fetch_result(): %d\n", err);
        }
        retries--;
    } while (err && retries);

    if (err == 0) {
        printk("Clock has started\n");
    }
    return err == 0;
}

#ifdef CONFIG_DEBUG_BELUGA_CLOCK
#define INIT_CLOCKS                                                            \
    do {                                                                       \
        bool retVal = clock_init(LOW_FREQ);                                    \
        while (!retVal)                                                        \
            ;                                                                  \
        retVal = clock_init(HIGH_FREQ);                                        \
        while (!retVal)                                                        \
            ;                                                                  \
    } while (0)
#else
#define INIT_CLOCKS                                                            \
    do {                                                                       \
    } while (0)
#endif

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

    if (tx_power == 1) {
        set_tx_power(true);
        printf("  TX Power: Max \r\n");
    } else {
        printf("  TX Power: Default \r\n");
    }
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
    printf("  Output Format: %s \r\n", (format == 1) ? "JSON" : "CSV");
}

static enum uwb_preamble_length load_data_rate(void) {
    enum uwb_datarate rate =
        (enum uwb_datarate)retrieveSetting(BELUGA_UWB_DATA_RATE);
    enum uwb_preamble_length length;
    switch (rate) {
    case UWB_DR_850K:
        printf("  UWB Data Rate: 850 kHz \r\n");
        break;
    case UWB_DR_110K:
        printf("  UWB Data Rate: 110 kHz \r\n");
        break;
    case UWB_DR_6M8:
    default:
        printf("  UWB Data Rate: 6.8MHz \r\n");
        rate = UWB_DR_6M8;
        break;
    }

    set_uwb_data_rate(rate, &length);
    return length;
}

static void load_preamble_length(enum uwb_preamble_length forcedLength) {
    int32_t length = retrieveSetting(BELUGA_UWB_PREAMBLE);
    enum uwb_preamble_length preambleLength = setting_to_preamble_enum(length);
    switch (preambleLength) {
    case UWB_PRL_64:
        printf("  UWB Preamble Length: 64 \r\n");
        break;
    case UWB_PRL_128:
        printf("  UWB Preamble Length: 128 \r\n");
        break;
    case UWB_PRL_256:
        printf("  UWB Preable Length: 256 \r\n");
        break;
    case UWB_PRL_512:
        printf("  UWB Preamble Length: 512 \r\n");
        break;
    case UWB_PRL_1024:
        printf("  UWB Preamble Length: 1024 \r\n");
        break;
    case UWB_PRL_2048:
        printf("  UWB Preamble Length: 2048 \r\n");
        break;
    case UWB_PRL_4096:
        printf("  UWB Preamble Length: 4096 \r\n");
        break;
    default:
        preambleLength = forcedLength;
        printf("  UWB Preamble Length: %" PRId32 " \r\n",
               preamble_length_to_setting(forcedLength));
        break;
    }

    set_uwb_preamble_length(preambleLength);
}

static void load_pulse_rate(void) {
    enum uwb_pulse_rate rate =
        (enum uwb_pulse_rate)retrieveSetting(BELUGA_UWB_PULSE_RATE);
    switch (rate) {
    case UWB_PR_16M:
        printf("  UWB Pulse Rate: 16MHz \r\n");
        break;
    case UWB_PR_64M:
    default:
        printf("  UWB Pulse Rate: 64MHz \r\n");
        rate = UWB_PR_64M;
        break;
    }

    set_pulse_rate(rate);
}

UNUSED static void load_power_amplifiers(void) {
    int32_t pwramp = retrieveSetting(BELUGA_RANGE_EXTEND);

    if (pwramp == 1) {
        enable_range_extension(false);
    } else {
        disable_range_extension(false);
    }
    printf("  Range Extension: %d \r\n", pwramp);
}

static void load_settings(void) {
    printf("Flash Configuration: \r\n");

    load_led_mode();
    load_id();
    load_bootmode();
    load_poll_rate();
    load_channel();
    enum uwb_preamble_length forced = load_data_rate();
    load_preamble_length(forced);
    load_pulse_rate();
    load_timeout();
    load_tx_power();
    load_stream_mode();
    load_twr_mode();
    load_out_format();
    if (IS_ENABLED(CONFIG_BELUGA_RANGE_EXTENSION)) {
        load_power_amplifiers();
    }
}

#if defined(CONFIG_BELUGA_RESET_REASON)
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

#define RESET_CAUSE                                                            \
    do {                                                                       \
        get_reset_cause();                                                     \
    } while (0)
#else
#define RESET_CAUSE                                                            \
    do {                                                                       \
    } while (0)
#endif

int main(void) {
    RESET_CAUSE;

    memset(seen_list, 0, ARRAY_SIZE(seen_list));

    INIT_CLOCKS;

    if (!init_voltage_regulator()) {
        printk("Failed to initialize voltage regulator\n");
        return 1;
    }

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

    if (!init_range_extension()) {
        printk("Failed to initialize range extension\n");
        return 1;
    }

    LED_INIT;

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

    printf("Node On: Firmware version %s\r\n", FIRMWARE_VERSION);

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
