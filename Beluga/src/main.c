/**
 * @file main.c
 * @brief Main application
 *
 * @date 6/19/2024
 * @author Tom Schmitz
 */

#include <beluga_message.h>
#include <serial/comms.h>

#include <app_leds.h>
#include <app_version.h>
#include <ble/ble_app.h>
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
#include <unistd.h>
#include <utils.h>
#include <voltage_regulator.h>
#include <watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/**
 * Logger for main
 */
LOG_MODULE_REGISTER(beluga_main, CONFIG_BELUGA_MAIN_LOG_LEVEL);

/**
 * Prints the settings header if not in frame mode
 *
 * @param[in] _comms Pointer to the comms instance
 */
#define SETTINGS_HEADER(_comms)                                                \
    do {                                                                       \
        if ((_comms)->ctx->format != FORMAT_FRAMES) {                          \
            struct beluga_msg msg = {.type = START_EVENT,                      \
                                     .payload.node_version =                   \
                                         "Node On: " APP_VERSION_STRING};      \
            comms_write_msg(_comms, &msg);                                     \
        }                                                                      \
    } while (0)

/**
 * Prints a formatted settings string
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _str The format string
 * @param[in] ... List of parameters to print
 */
#define _SETTINGS_PRINT_FMT(_comms, _str, ...)                                 \
    do {                                                                       \
        struct beluga_msg msg = {.type = START_EVENT};                         \
        char s[256];                                                           \
        snprintf(s, sizeof(s) - 1, "  " _str, __VA_ARGS__);                    \
        msg.payload.node_version = s;                                          \
        comms_write_msg(_comms, &msg);                                         \
    } while (0)

/**
 * Prints a setting
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _str The settings string
 */
#define __SETTINGS_PRINT(_comms, _str)                                         \
    do {                                                                       \
        struct beluga_msg msg = {.type = START_EVENT,                          \
                                 .payload.node_version = ("  " _str)};         \
        comms_write_msg(_comms, &msg);                                         \
    } while (0)

/**
 * Prints a setting
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _str The setting string to print
 * @param[in] ... Optional list of arguments to print
 */
#define _SETTINGS_PRINT(_comms, _str, ...)                                     \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (__SETTINGS_PRINT(_comms, _str)),       \
                (_SETTINGS_PRINT_FMT(_comms, _str, __VA_ARGS__)))

/**
 * Prints a setting if not in frame mode
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _str The setting string to print
 * @param[in] ... Optional list of arguments to print
 */
#define SETTINGS_PRINT(_comms, _str, ...)                                      \
    do {                                                                       \
        if ((_comms)->ctx->format != FORMAT_FRAMES) {                          \
            _SETTINGS_PRINT(_comms, _str, __VA_ARGS__);                        \
        }                                                                      \
    } while (0)

/**
 * Print the node version if in framed mode, otherwise, prints a line break
 * if not in framed mode.
 *
 * @param[in] _comms Pointer to the comms instance
 */
#define SETTINGS_BREAK(_comms)                                                 \
    do {                                                                       \
        if ((_comms)->ctx->format == FORMAT_FRAMES) {                          \
            struct beluga_msg msg = {.type = START_EVENT,                      \
                                     .payload.node_version =                   \
                                         APP_VERSION_STRING};                  \
            comms_write_msg(_comms, &msg);                                     \
        } else {                                                               \
            struct beluga_msg msg = {.type = START_EVENT,                      \
                                     .payload.node_version = "\n"};            \
            comms_write_msg(_comms, &msg);                                     \
        }                                                                      \
    } while (0)

/**
 * Runs a custom print setting routine with arguments
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] callback The custom print setting routine to run
 * @param[in] ... Argument list
 */
#define _CUSTOM_INIT_MSG_ARGS(_comms, callback, ...)                           \
    callback(_comms, __VA_ARGS__)

/**
 * Runs a custom print setting routine without arguments
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] callback The custom print setting routine to run
 */
#define _CUSTOM_INIT_MSG_NOARGS(_comms, callback) callback(_comms)

/**
 * Runs a custom print setting routine
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] callback The custom print setting routine to run
 * @param[in] ... Additional arguments for the routine
 */
#define _CUSTOM_INIT_MSG(_comms, callback, ...)                                \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__),                                         \
                (_CUSTOM_INIT_MSG_NOARGS(_comms, callback)),                   \
                (_CUSTOM_INIT_MSG_ARGS(_comms, callback, __VA_ARGS__)))

/**
 * Runs a custom print setting routine if not in framed mode
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] callback The custom print setting routine to run
 * @param[in] ... Additional arguments for the routine
 */
#define CUSTOM_INIT_MSG(_comms, callback, ...)                                 \
    do {                                                                       \
        if ((_comms)->ctx->format != FORMAT_FRAMES) {                          \
            _CUSTOM_INIT_MSG(_comms, callback, __VA_ARGS__);                   \
        }                                                                      \
    } while (0)

#if CONFIG_STATUS_BLINK_PERIOD != 0
/**
 * Work handler for blinking the status (Power) LED.
 * @param[in] work Pointer to the work item.
 */
static void blink_status_led(struct k_work *work) {
    struct k_work_delayable *dwork =
        CONTAINER_OF(work, struct k_work_delayable, work);
    static enum led_state state = LED_ON;
    static uint32_t error_blink = 0;
    k_timeout_t schedule_period = K_MSEC(CONFIG_STATUS_BLINK_PERIOD);
    const struct comms *comms = comms_backend_uart_get_ptr();

    if (comms_check_rx_error(comms)) {
        if ((state == LED_ON && error_blink == 0) ||
            (state == LED_OFF && error_blink == 1)) {
            schedule_period = K_MSEC(200);
            error_blink++;
        } else {
            error_blink = 0;
        }
    }

    update_led_state(LED_POWER, state);
    state = (state == LED_ON) ? LED_OFF : LED_ON;

    k_work_schedule(dwork, schedule_period);
}
K_WORK_DELAYABLE_DEFINE(blink_work, blink_status_led);
#endif // CONFIG_STATUS_BLINK_PERIOD != 0

/**
 * Load the LED mode from the settings and display the current state
 */
static void load_led_mode(const struct comms *comms) {
    int32_t led_mode = retrieveSetting(BELUGA_LEDMODE);

    if (led_mode == 1) {
        all_leds_off();
    }
#if CONFIG_STATUS_BLINK_PERIOD != 0
    k_work_schedule(&blink_work, K_NO_WAIT);
#else
    update_led_state(LED_POWER, LED_ON);
#endif // CONFIG_STATUS_BLINK_PERIOD != 0
    SETTINGS_PRINT(comms, "LED Mode: %d", led_mode);
}

/**
 * Load the node ID from settings and display it
 */
static void load_id(const struct comms *comms) {
    int32_t nodeID = retrieveSetting(BELUGA_ID);

    if (nodeID != DEFAULT_ID_SETTING) {
        update_node_id((uint16_t)nodeID);
        set_initiator_id((uint16_t)nodeID);
        set_responder_id((uint16_t)nodeID);
        SETTINGS_PRINT(comms, "Node ID: %d", nodeID);
    } else {
        SETTINGS_PRINT(comms, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        SETTINGS_PRINT(comms, "!Warning! Please setup node ID!");
        SETTINGS_PRINT(comms, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
}

/**
 * Load the mode the node should boot in, update the states for the node, and
 * display the boot mode
 */
static void load_bootmode(const struct comms *comms) {
    int32_t bootMode = retrieveSetting(BELUGA_BOOTMODE);

    switch (bootMode) {
    case 1:
        k_sem_give(&print_list_sem);
        enable_bluetooth();
        update_led_state(LED_BLE, LED_ON);
        break;
    case 2:
        k_sem_give(&print_list_sem);
        enable_bluetooth();
        update_led_state(LED_BLE, LED_ON);
        update_uwb_state(true);
    case 0:
        break;
    default:
        SETTINGS_PRINT(comms, "Boot Mode: Unknown");
        break;
    }
    SETTINGS_PRINT(comms, "Boot Mode: %d", bootMode);
}

/**
 * Retrieve the polling rate, set it, and display what it is
 */
static void load_poll_rate(const struct comms *comms) {
    int32_t rate = retrieveSetting(BELUGA_POLL_RATE);

    // Account for new versions of the FW
    if (rate > (int32_t)CONFIG_MAX_POLLING_RATE) {
        rate = CONFIG_MAX_POLLING_RATE;
        updateSetting(BELUGA_POLL_RATE, rate);
    }

    set_rate(rate);
    SETTINGS_PRINT(comms, "UWB Polling Rate: %d", rate);
}

/**
 * Retrieve the UWB channel, set it, and display what it is
 */
static void load_channel(const struct comms *comms) {
    int32_t channel = retrieveSetting(BELUGA_UWB_CHANNEL);
    set_uwb_channel(channel);
    SETTINGS_PRINT(comms, "UWB Channel: %d", channel);
}

/**
 * Retrieve the node timeout, set it, and display what it is
 */
static void load_timeout(const struct comms *comms) {
    int32_t timeout = retrieveSetting(BELUGA_BLE_TIMEOUT);
    set_node_timeout(timeout);
    SETTINGS_PRINT(comms, "BLE Timeout: %d", timeout);
}

/**
 * Retrieve the UWB TX power, set it, and display what it is
 */
static void load_tx_power(const struct comms *comms) {
    int32_t tx_power = retrieveSetting(BELUGA_TX_POWER);
    struct uwb_tx_power_config config = {
        .mode = UWB_TX_PWR_CONFIG_RAW,
        .raw_power = tx_power,
    };

    set_tx_power(&config);
    CUSTOM_INIT_MSG(comms, print_tx_power, (uint32_t)tx_power);
}

/**
 * Retrieve the neighbor streaming mode, set it, and display the current mode
 */
static void load_stream_mode(const struct comms *comms) {
    int32_t stream_mode = retrieveSetting(BELUGA_STREAMMODE);
    set_stream_mode(stream_mode != 0);
    SETTINGS_PRINT(comms, "Stream Mode: %d", stream_mode);
}

/**
 * Retrieve the two-way ranging mode, set it, and display what it is
 */
static void load_twr_mode(const struct comms *comms) {
    int32_t twr = retrieveSetting(BELUGA_TWR);
    set_twr_mode(twr != 0);
    SETTINGS_PRINT(comms, "Ranging Mode: %d", twr);
}

/**
 * Retrieve the output format mode, set it, and display what it is
 */
static void load_out_format(const struct comms *comms) {
    CUSTOM_INIT_MSG(comms, print_format);
}

/**
 * Retrieve the UWB PHR mode, set it, and display what it is
 */
static void load_phr_mode(const struct comms *comms) {
    int32_t mode = retrieveSetting(BELUGA_UWB_PHR);
    SETTINGS_PRINT(comms, "UWB PHR Mode: %" PRId32, mode);
    uwb_set_phr_mode((enum uwb_phr_mode)mode);
}

/**
 * Retrieve the UWB data rate, set it, and display what it is
 */
static void load_data_rate(const struct comms *comms) {
    enum uwb_datarate rate =
        (enum uwb_datarate)retrieveSetting(BELUGA_UWB_DATA_RATE);
    CUSTOM_INIT_MSG(comms, rate = print_uwb_datarate, rate);
    uwb_set_datarate(rate);
}

/**
 * Retrieve the UWB pulse repetition rate, set it, and display what it is
 */
static void load_pulse_rate(const struct comms *comms) {
    enum uwb_pulse_rate rate =
        (enum uwb_pulse_rate)retrieveSetting(BELUGA_UWB_PULSE_RATE);
    CUSTOM_INIT_MSG(comms, rate = print_pulse_rate, rate);
    uwb_set_pulse_rate((enum uwb_pulse_rate)rate);
}

/**
 * Retrieve the UWB preamble length, set it, and display what it is
 */
static void load_preamble_length(const struct comms *comms) {
    int32_t length = retrieveSetting(BELUGA_UWB_PREAMBLE);
    SETTINGS_PRINT(comms, "UWB Preamble Length: %" PRId32, length);
    uwb_set_preamble((enum uwb_preamble_length)length);
}

/**
 * Retrieve the UWB PAC size, set it, and display what it is
 */
static void load_pac_size(const struct comms *comms) {
    int32_t pac = retrieveSetting(BELUGA_UWB_PAC);
    CUSTOM_INIT_MSG(comms, pac = print_pac_size, pac);
    set_pac_size((enum uwb_pac)pac);
}

/**
 * Retrieve the SFD mode, set it, and display what it is
 */
static void load_sfd_mode(const struct comms *comms) {
    int32_t mode = retrieveSetting(BELUGA_UWB_NSSFD);
    SETTINGS_PRINT(comms, "UWB Nonstandard SFD Length: %" PRId32, mode);
    set_sfd_mode((enum uwb_sfd)mode);
}

/**
 * Retrieve the UWB PAN ID, set it, and display what it is
 */
static void load_pan_id(const struct comms *comms) {
    int32_t pan_id = retrieveSetting(BELUGA_PAN_ID);
    CUSTOM_INIT_MSG(comms, print_pan_id, pan_id);
    set_uwb_pan_id((uint16_t)pan_id);
}

/**
 * Retrieve the power amplifier state, set it, and display what it is
 */
UNUSED static void load_power_amplifiers(const struct comms *comms) {
    int ret;
    int32_t pwramp = retrieveSetting(BELUGA_RANGE_EXTEND);

    ret = update_power_mode((enum power_mode)pwramp);

    if (pwramp == 0 || ret != 0) {
        update_led_state(LED_PWRAMP, LED_OFF);
    } else {
        update_led_state(LED_PWRAMP, LED_ON);
    }

    SETTINGS_PRINT(comms, "Range Extension: %d", pwramp);
}

/**
 * Load the format without printing the format value
 * @param[in] comms Pointer to the comms instance
 */
static void load_format_no_msg(const struct comms *comms) {
    int32_t format = retrieveSetting(BELUGA_OUT_FORMAT);
    set_format(comms, (enum comms_out_format_mode)format);
}

#if IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT)
/**
 * Loads the eviction scheme setting and prints what eviction policy is
 * in use.
 * @param[in] comms Pointer to the comms instance
 */
UNUSED static void load_eviction_scheme(const struct comms *comms) {
    int32_t scheme = retrieveSetting(BELUGA_EVICTION_SCHEME);

    set_node_eviction_policy(scheme);
    CUSTOM_INIT_MSG(comms, print_eviction_scheme);
}
#else
/**
 * Loads the eviction scheme setting and prints what eviction policy is
 * in use.
 * @param[in] comms Pointer to the comms instance
 */
#define load_eviction_scheme(...) (void)0
#endif

/**
 * Load the verbosity setting
 * @param[in] comms Pointer to the comms object
 */
static void load_verbose(const struct comms *comms) {
    int32_t verbose = retrieveSetting(BELUGA_VERBOSE);
    set_verbosity(comms, verbose == 1);
}

/**
 * Load the antenna calibrations. No need to print the calibration values.
 */
static void load_antenna_calibrations(void) {
    int32_t calibration = retrieveSetting(BELUGA_RX_ANT_DELAY_16);
    set_initiator_antenna_rx_delay(UWB_PR_16M, calibration);
    set_responder_antenna_rx_delay(UWB_PR_16M, calibration);

    calibration = retrieveSetting(BELUGA_RX_ANT_DELAY_64);
    set_initiator_antenna_rx_delay(UWB_PR_64M, calibration);
    set_responder_antenna_rx_delay(UWB_PR_64M, calibration);

    calibration = retrieveSetting(BELUGA_TX_ANT_DELAY_16);
    set_initiator_antenna_tx_delay(UWB_PR_16M, calibration);
    set_responder_antenna_tx_delay(UWB_PR_16M, calibration);

    calibration = retrieveSetting(BELUGA_TX_ANT_DELAY_64);
    set_initiator_antenna_tx_delay(UWB_PR_64M, calibration);
    set_responder_antenna_tx_delay(UWB_PR_64M, calibration);
}

/**
 * Load all the settings, initialize all the states, and display what the
 * settings are
 */
static void load_settings(const struct comms *comms) {
    load_format_no_msg(comms);

    SETTINGS_HEADER(comms);
    SETTINGS_PRINT(comms, "Flash Configuration:");

    load_led_mode(comms);
    load_id(comms);
    load_bootmode(comms);

    // Disable UWB LED since setters check LED if UWB is active or not
    enum led_state uwb_state = get_uwb_led_state();
    update_led_state(LED_UWB, LED_OFF);

    // Load UWB settings since ranging task has not started yet
    load_poll_rate(comms);
    load_channel(comms);
    load_phr_mode(comms);
    load_data_rate(comms);
    load_pulse_rate(comms);
    load_preamble_length(comms);
    load_pac_size(comms);
    load_sfd_mode(comms);
    load_pan_id(comms);
    load_antenna_calibrations();

    // Restore UWB state in the LEDs
    update_led_state(LED_UWB, uwb_state);

    load_timeout(comms);
    load_tx_power(comms);
    load_stream_mode(comms);
    load_twr_mode(comms);
    load_out_format(comms);
    if (IS_ENABLED(CONFIG_BELUGA_RANGE_EXTENSION)) {
        load_power_amplifiers(comms);
    }
    load_eviction_scheme(comms);
    load_verbose(comms);

    SETTINGS_BREAK(comms);
}

/**
 * Updates the UWB transmit power and saves the setting to flash.
 * @param[in] power The new TX power
 */
static void update_uwb_tx_power(uint32_t power) {
    struct uwb_tx_power_config config;
    config.mode = UWB_TX_PWR_CONFIG_RAW;
    config.raw_power = power;
    (void)set_tx_power(&config);
    updateSetting(BELUGA_TX_POWER, (int32_t)power);
}

/**
 * Updates the UWB preamble and saves the setting to flash.
 * @param[in] preamble The new preamble
 */
static void update_uwb_preamble(uint16_t preamble) {
    uwb_set_preamble(preamble);
    updateSetting(BELUGA_UWB_PREAMBLE, preamble);
}

/**
 * Updates the UWB channel and saves the setting to flash.
 * @param[in] channel The new channel.
 */
static void update_uwb_channel(uint8_t channel) {
    set_uwb_channel(channel);
    updateSetting(BELUGA_UWB_CHANNEL, channel);
}

/**
 * Updates the UWB ranging protocol and saves the setting to flash.
 * @param[in] twr The new ranging protocol
 */
static void update_uwb_twr(bool twr) {
    set_twr_mode(twr);
    updateSetting(BELUGA_TWR, twr);
}

/**
 * Updates the UWB PHR and saves the setting to flash.
 * @param[in] phr The new PHR
 */
static void update_uwb_phr(bool phr) {
    uwb_set_phr_mode(phr);
    updateSetting(BELUGA_UWB_PHR, phr);
}

/**
 * Updates the UWB SFD and saves the settings to flash.
 * @param[in] sfd THe new SFD
 */
static void update_uwb_sfd(bool sfd) {
    set_sfd_mode(sfd);
    updateSetting(BELUGA_UWB_NSSFD, sfd);
}

/**
 * Updates the UWB data rate and saves the setting to flash
 * @param[in] rate The new data rate
 */
static void update_uwb_data_rate(uint8_t rate) {
    uwb_set_datarate(rate);
    updateSetting(BELUGA_UWB_DATA_RATE, rate);
}

/**
 * Updates the UWB PAC and saves the setting to flash
 * @param[in] pac The new PAC
 */
static void update_uwb_pac(uint8_t pac) {
    set_pac_size(pac);
    updateSetting(BELUGA_UWB_PAC, pac);
}

/**
 * Updates the UWB pulse rate and saves the setting to flash
 * @param[in] pulse_rate The new pulse rate
 */
static void update_uwb_pulse_rate(bool pulse_rate) {
    uwb_set_pulse_rate(pulse_rate);
    updateSetting(BELUGA_UWB_PULSE_RATE, pulse_rate);
}

/**
 * Updates the external power amp mode and saves the setting to flash.
 * @param[in] pwramp The new power amp mode
 */
static void update_power_mode_(uint8_t pwramp) {
    update_power_mode(pwramp);
    updateSetting(BELUGA_RANGE_EXTEND, pwramp);
}

static void finalize_serial_setup(const struct comms *comms) {
    int32_t block = retrieveSetting(BELUGA_WAIT_USB_HOST);
    set_wait_usb_host(comms, block != 0);

    if (block != 0) {
        wait_comms_ready(comms);
    }
}

/**
 * @brief Main entry point of the application
 * @return 1 on error
 */
int main(void) {
    const struct comms *comms = comms_backend_uart_get_ptr();
    RESET_CAUSE();

    memset(seen_list, 0, sizeof(seen_list));

    INIT_CLOCKS();

    if (init_voltage_regulator() != 0) {
        LOG_ERR("Failed to initialize voltage regulator");
        return 1;
    }

    if (init_bt_stack() != 0) {
        LOG_ERR("Failed to init bluetooth stack");
        return 1;
    }

    if (initBelugaSettings() != 0) {
        LOG_ERR("Unable to init flash");
        return 1;
    }

    if (init_spi1() < 0) {
        LOG_ERR("Failed to initialize SPI 1");
        return 1;
    }

    if (init_range_extension() != 0) {
        LOG_ERR("Failed to initialize range extension");
        return 1;
    }

    LED_INIT();

    if (configure_watchdog_timer() < 0) {
        LOG_ERR("Failed to configure watchdog timer");
        return 1;
    }

    LOG_INF("Module initialization done. Waiting for DTR to be asserted");
    finalize_serial_setup(comms);

    init_uwb();

    load_settings(comms);

    init_responder_thread();
    init_print_list_task();
    init_ranging_thread();
    init_monitor_thread();

    for (;;) {
        enum led_state uwb_led_state;
        k_poll(&sync_configs.ready, 1, K_FOREVER);
        wait_ble_disconnect(K_FOREVER);
        uwb_led_state = get_uwb_led_state();
        if (uwb_led_state == LED_ON) {
            update_uwb_state(false);
        }

        update_uwb_tx_power(sync_configs.config.TX_POWER);
        update_uwb_preamble(sync_configs.config.PREAMBLE);
        update_uwb_channel(sync_configs.config.CHANNEL);
        update_uwb_twr(sync_configs.config.TWR);
        update_uwb_phr(sync_configs.config.PHR);
        update_uwb_sfd(sync_configs.config.SFD);
        update_uwb_data_rate(sync_configs.config.DATA_RATE);
        update_uwb_pac(sync_configs.config.PAC);
        update_uwb_pulse_rate(sync_configs.config.PULSE_RATE);
        update_power_mode_(sync_configs.config.POWER_AMP);

        if (uwb_led_state == LED_ON) {
            update_uwb_state(true);
        }
        k_poll_signal_reset(&sync_configs.ready_sig);
    }

    return 1;
}
