/**
 * @file main.c
 * @brief Main application
 *
 * @date 6/19/2024
 * @author Tom Schmitz
 */

#include "ble_app.h"
#include <app_leds.h>
#include <app_version.h>
#include <debug.h>
#include <initiator.h>
#include <led_config.h>
#include <list_monitor.h>
#include <list_neighbors.h>
#include <range_extension.h>
#include <ranging.h>
#include <responder.h>
#include <serial/comms.h>
#include <settings.h>
#include <spi.h>
#include <unistd.h>
#include <utils.h>
#include <voltage_regulator.h>
#include <watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main_app, CONFIG_BELUGA_MAIN_LOG_LEVEL);

#if defined(CONFIG_BELUGA_FRAMES)
#include <beluga_message.h>

#define INIT_MSG(...)                   (void)0
#define CUSTOM_INIT_MSG(callback_, ...) callback_(__VA_ARGS__)
#define NODE_VERSION_BANNER()           (void)0
#define FW_CONFIG_BANNER()              (void)0
#define SETTINGS_BREAK()                                                       \
    do {                                                                       \
        const struct comms *comms = comms_backend_uart_get_ptr();              \
        struct beluga_msg msg = {.type = START_EVENT,                          \
                                 .payload.node_version = APP_VERSION_STRING};  \
        (void)write_message_frame(comms, &msg);                                \
    } while (0)

#else
#define _INIT_MSG(str)           printf("  " str "\n")
#define _INIT_MSG_ARGS(str, ...) printf("  " str "\n", __VA_ARGS__)
#define INIT_MSG(format_str, ...)                                              \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (_INIT_MSG(format_str)),                \
                (_INIT_MSG_ARGS(format_str, __VA_ARGS__)))
#define CUSTOM_INIT_MSG(callback_, ...)                                        \
    do {                                                                       \
        printf("  ");                                                          \
        callback_(__VA_ARGS__);                                                \
        printf("\n");                                                          \
    } while (0)

#define NODE_VERSION_BANNER() printf("Node On: " APP_VERSION_STRING "\n")
#define FW_CONFIG_BANNER()    printf("Flash Configuration: \n");
#define SETTINGS_BREAK()      printf("\n")
#endif

/**
 * Load the LED mode from the settings and display the current state
 */
static void load_led_mode(void) {
    int32_t led_mode = retrieveSetting(BELUGA_LEDMODE);

    if (led_mode == 1) {
        all_leds_off();
    }
    update_led_state(LED_POWER_ON);
    INIT_MSG("LED Mode: %d", led_mode);
}

/**
 * Load the node ID from settings and display it
 */
static void load_id(void) {
    int32_t nodeID = retrieveSetting(BELUGA_ID);

    if (nodeID != DEFAULT_ID_SETTING) {
        update_node_id((uint16_t)nodeID);
        set_initiator_id((uint16_t)nodeID);
        set_responder_id((uint16_t)nodeID);
        INIT_MSG("Node ID: %d", nodeID);
    } else {
        INIT_MSG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        INIT_MSG("!Warning! Please setup node ID!");
        INIT_MSG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
}

/**
 * Load the mode the node should boot in, update the states for the node, and
 * display the boot mode
 */
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
        INIT_MSG("Boot Mode: Unknown");
        break;
    }
    INIT_MSG("Boot Mode: %d", bootMode);
}

/**
 * Retrieve the polling rate, set it, and display what it is
 */
static void load_poll_rate(void) {
    int32_t rate = retrieveSetting(BELUGA_POLL_RATE);
    set_rate(rate);
    advertising_reconfig(rate != 0);
    INIT_MSG("UWB Polling Rate: %d", rate);
}

/**
 * Retrieve the UWB channel, set it, and display what it is
 */
static void load_channel(void) {
    int32_t channel = retrieveSetting(BELUGA_UWB_CHANNEL);
    set_uwb_channel(channel);
    INIT_MSG("UWB Channel: %d", channel);
}

/**
 * Retrieve the node timeout, set it, and display what it is
 */
static void load_timeout(void) {
    int32_t timeout = retrieveSetting(BELUGA_BLE_TIMEOUT);
    set_node_timeout(timeout);
    INIT_MSG("BLE Timeout: %d", timeout);
}

/**
 * Retrieve the UWB TX power, set it, and display what it is
 */
static void load_tx_power(void) {
    int32_t tx_power = retrieveSetting(BELUGA_TX_POWER);
    set_tx_power((uint32_t)tx_power);
    CUSTOM_INIT_MSG(print_tx_power, (uint32_t)tx_power);
}

/**
 * Retrieve the neighbor streaming mode, set it, and display the current mode
 */
static void load_stream_mode(void) {
    int32_t stream_mode = retrieveSetting(BELUGA_STREAMMODE);
    set_stream_mode(stream_mode != 0);
    INIT_MSG("Stream Mode: %d", stream_mode);
}

/**
 * Retrieve the two-way ranging mode, set it, and display what it is
 */
static void load_twr_mode(void) {
    int32_t twr = retrieveSetting(BELUGA_TWR);
    set_twr_mode(twr != 0);
    INIT_MSG("Ranging Mode: %d", twr);
}

/**
 * Retrieve the output format mode, set it, and display what it is
 */
static void load_out_format(void) {
    int32_t format = retrieveSetting(BELUGA_OUT_FORMAT);
    set_format_mode(format == 1);
    CUSTOM_INIT_MSG(print_output_format, format);
}

/**
 * Retrieve the UWB PHR mode, set it, and display what it is
 */
static void load_phr_mode(void) {
    int32_t mode = retrieveSetting(BELUGA_UWB_PHR);
    INIT_MSG("UWB PHR Mode: %" PRId32, mode);
    uwb_set_phr_mode((enum uwb_phr_mode)mode);
}

/**
 * Retrieve the UWB data rate, set it, and display what it is
 */
static void load_data_rate(void) {
    enum uwb_datarate rate =
        (enum uwb_datarate)retrieveSetting(BELUGA_UWB_DATA_RATE);
    CUSTOM_INIT_MSG(rate = print_uwb_datarate, rate);
    uwb_set_datarate(rate);
}

/**
 * Retrieve the UWB pulse repetition rate, set it, and display what it is
 */
static void load_pulse_rate(void) {
    enum uwb_pulse_rate rate =
        (enum uwb_pulse_rate)retrieveSetting(BELUGA_UWB_PULSE_RATE);
    CUSTOM_INIT_MSG(rate = print_pulse_rate, rate);
    uwb_set_pulse_rate((enum uwb_pulse_rate)rate);
}

/**
 * Retrieve the UWB preamble length, set it, and display what it is
 */
static void load_preamble_length(void) {
    int32_t length = retrieveSetting(BELUGA_UWB_PREAMBLE);
    INIT_MSG("UWB Preamble Length: %" PRId32, length);
    uwb_set_preamble((enum uwb_preamble_length)length);
}

/**
 * Retrieve the UWB PAC size, set it, and display what it is
 */
static void load_pac_size(void) {
    int32_t pac = retrieveSetting(BELUGA_UWB_PAC);
    CUSTOM_INIT_MSG(pac = print_pac_size, pac);
    set_pac_size((enum uwb_pac)pac);
}

/**
 * Retrieve the SFD mode, set it, and display what it is
 */
static void load_sfd_mode(void) {
    int32_t mode = retrieveSetting(BELUGA_UWB_NSSFD);
    INIT_MSG("UWB Nonstandard SFD Length: %" PRId32, mode);
    set_sfd_mode((enum uwb_sfd)mode);
}

/**
 * Retrieve the UWB PAN ID, set it, and display what it is
 */
static void load_pan_id(void) {
    int32_t pan_id = retrieveSetting(BELUGA_PAN_ID);
    CUSTOM_INIT_MSG(print_pan_id, pan_id);
    set_initiator_pan_id((uint16_t)pan_id);
    set_responder_pan_id((uint16_t)pan_id);
}

/**
 * Retrieve the power amplifier state, set it, and display what it is
 */
UNUSED static void load_power_amplifiers(void) {
    int ret;
    int32_t pwramp = retrieveSetting(BELUGA_RANGE_EXTEND);

    if (pwramp == 0) {
        update_power_mode(POWER_MODE_BYPASS);
    } else if (pwramp == 1) {
        ret = update_power_mode(POWER_MODE_LOW);
    } else {
        ret = update_power_mode(POWER_MODE_HIGH);
    }

    if (pwramp == 0 || ret != 0) {
        update_led_state(LED_PWRAMP_OFF);
    } else {
        update_led_state(LED_PWRAMP_ON);
    }

    INIT_MSG("Range Extension: %d", pwramp);
}

/**
 * Load all the settings, initialize all the states, and display what the
 * settings are
 */
static void load_settings(void) {
    NODE_VERSION_BANNER();
    FW_CONFIG_BANNER();

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
    SETTINGS_BREAK();
}

/**
 * @brief Main entry point of the application
 * @return 1 on error
 */
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
        return 1;
    }

    init_uwb();

    load_settings();

    kill_task_watchdog(&task_watchdog);

    init_responder_thread();
    init_print_list_task();
    init_ranging_thread();
    init_monitor_thread();

    for (;;) {
        k_sleep(K_FOREVER);
    }

    return 1;
}
