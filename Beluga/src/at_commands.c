/**
 * @file   at_commands.c
 *
 * @brief  Beluga AT commands
 *
 * This defines all the AT commands available on Beluga. It not only defines
 * the AT commands, but also parses the arguments, provides argument checking
 * helpers, and runs the commands thread.
 *
 *  @date   6/1/2024
 *
 *  @author Tom Schmitz
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

#include "deca_types.h"
#include <initiator.h>
#include <responder.h>

#include <at_commands.h>
#include <ble_app.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <uart.h>

#include <app_leds.h>
#include <list_monitor.h>
#include <list_neighbors.h>
#include <power_manager.h>
#include <range_extension.h>
#include <ranging.h>
#include <serial/comms.h>
#include <settings.h>
#include <stdio.h>
#include <stdlib.h>
#include <utils.h>
#include <watchdog.h>
#include <zephyr/logging/log.h>

/**
 * Logger for the AT commands
 */
LOG_MODULE_REGISTER(at_commands, CONFIG_AT_COMMANDS_LOG_LEVEL);

// #include <beluga_message.h>
//
// #def ine  _OK \
//    do { \
//        struct beluga_msg msg = {.type = COMMAND_RESPONSE, \
//                                 .payload.response = "OK"}; \
//        (void)write_message_frame(&msg); \
//    } while (0)
//
// #def ine  _OK_MSG(_msg) \
//    do { \
//        struct beluga_msg msg = {.type = COMMAND_RESPONSE, \
//                                 .payload.response = _msg " OK"}; \
//        (void)write_message_frame(&msg); \
//    } while (0)
//
// #def ine  _OK_MSG_ARGS(_msg, ...) \
//    do { \
//        uint8_t format_buf[128]; \
//        struct beluga_msg msg = {.type = COMMAND_RESPONSE, \
//                                 .payload.response = format_buf}; \
//        snprintf(format_buf, sizeof(format_buf), _msg " OK", __VA_ARGS__); \
//        (void)write_message_frame(&msg); \
//    } while (0)
//
// #def ine _OK_MSG_ (msg, ...) \
//    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (_OK_MSG(msg)), \
//                (_OK_MSG_ARGS(msg, __VA_ARGS__)))
//
// #def ine OK(...) \
//    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (_OK), \
//                (_OK_MSG_(GET_ARG_N(1, __VA_ARGS__), \
//                          GET_ARGS_LESS_N(1, __VA_ARGS__))))
//
// #def ine  _ERROR(_msg) \
//    do { \
//        struct beluga_msg msg = {.type = COMMAND_RESPONSE, \
//                                 .payload.response = (_msg)}; \
//        (void)write_message_frame(&msg); \
//    } while (0)
// #def ine  _ERROR_ARGS(_msg, ...) \
//    do { \
//        uint8_t format_buf[256]; \
//        struct beluga_msg msg = {.type = COMMAND_RESPONSE, \
//                                 .payload.response = format_buf}; \
//        snprintf(format_buf, sizeof(format_buf), (_msg), __VA_ARGS__); \
//        (void)write_message_frame(&msg); \
//    } while (0)
//
// #def ine ERROR(ms g, ...) \
//    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (_ERROR(msg)), \
//                (_ERROR_ARGS(msg, __VA_ARGS__)))

/**
 * Determines the maximum amount of tokens a string can get parsed into
 */
#define MAX_TOKENS 20

/**
 * @brief Checks if the minimum amount of arguments have been passed in. If not,
 * prints an error message and directs the caller to return early.
 *
 * @param[in] argc The amount of tokens that were parsed
 * @param[in] required The required amount of tokens needed for the function to
 * work properly
 */
#define CHECK_ARGC(_comms, argc, required)                                     \
    do {                                                                       \
        if ((uint16)(argc) < (uint16_t)(required)) {                           \
            ERROR(_comms, "Missing argument(s)");                              \
            return;                                                            \
        }                                                                      \
    } while (0)

/**
 * @brief Prints out the saved setting if the minimum amount of tokens have not
 * been passed in
 *
 * @param[in] argc The amount of tokens parsed
 * @param[in] required The amount of tokens required to do something other than
 * "read the setting"
 * @param[in] setting The enum that defines the setting
 * @param[in] settingstr The string representation of the setting
 */
#define READ_SETTING_DEFAULT(_comms, argc, required, setting, settingstr)      \
    do {                                                                       \
        if ((uint16_t)(argc) < (uint16_t)(required)) {                         \
            int32_t _setting = retrieveSetting(setting);                       \
            OK(_comms, settingstr ": %" PRIu32, (uint32_t)_setting);           \
        }                                                                      \
    } while (0)

/**
 * @brief Prints out the saved setting if the minimum amount of tokens have not
 * been passed in
 *
 * @param[in] argc The amount of tokens parsed
 * @param[in] required The amount of tokens required to do something other than
 * "read the setting"
 * @param[in] setting The enum that defines the setting
 * @param[in] settingstr The string representation of the setting
 * @param[in] callback The custom print function for the setting
 */
#define READ_SETTING_CALLBACK(_comms, argc, required, setting, settingstr,     \
                              callback)                                        \
    do {                                                                       \
        if (get_format_mode()) {                                               \
            READ_SETTING_DEFAULT(_comms, argc, required, setting, settingstr); \
        } else if ((uint16_t)(argc) < (uint16_t)(required)) {                  \
            int32_t _setting = retrieveSetting(setting);                       \
            callback(_setting);                                                \
            OK(_comms);                                                        \
        }                                                                      \
    } while (0)

/**
 * @brief Prints out the saved setting if the minimum amount of tokens have not
 * been passed in
 *
 * @param[in] argc The amount of tokens parsed
 * @param[in] required The amount of tokens required to do something other than
 * "read the setting"
 * @param[in] setting The enum that defines the setting
 * @param[in] settingstr The string representation of the setting
 * @param[in] callback An optional custom print function for the setting
 */
#define READ_SETTING(_comms, argc, required, setting, settingstr, callback...) \
    COND_CODE_1(                                                               \
        IS_EMPTY(callback),                                                    \
        (READ_SETTING_DEFAULT(_comms, argc, required, setting, settingstr)),   \
        (READ_SETTING_CALLBACK(_comms, argc, required, setting, settingstr,    \
                               GET_ARG_N(1, callback))))

/**
 * Converts an integer into a boolean given that the integer is a valid value.
 *
 * @param[out] boolarg The boolean representation of the integer
 * @param[in] intarg The integer to convert into a boolean
 *
 * @return true if the conversion was successful
 * @return false if the conversion was not successful
 */
STATIC_INLINE bool int2bool(bool *boolarg, int32_t intarg) {
    if (intarg == 0) {
        *boolarg = false;
    } else if (intarg == 1) {
        *boolarg = true;
    } else {
        return false;
    }
    return true;
}

/**
 * @brief Converts a string to a signed 32-bit integer assuming the string
 * representation is base 10 or base 16
 *
 * @param[in] str The string to convert into an integer
 * @param[out] result The resulting integer parsed from the string
 *
 * @return true if the integer was parsed successfully without any discrepancies
 * @return false if the integer is out of range or the entire string is not an
 * integer
 */
static bool strtoint32(const char *str, int32_t *result) {
    char *endptr;
    unsigned long ret;
    int base = 10;
    char *start = (char *)str;

    if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X')) {
        base = 16;
        start += 2;
    }

    errno = 0;
    ret = strtol(start, &endptr, base);

    if (errno == ERANGE || (int64_t)ret > (int64_t)INT32_MAX ||
        (int64_t)ret < (int64_t)INT32_MIN || isgraph((int)*endptr)) {
        *result = 0;
        return false;
    }

    *result = (int32_t)ret;
    return true;
}

/**
 * The STARTUWB AT command
 *
 * This will start UWB ranging
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(STARTUWB) {
    LOG_INF("Running STARTUWB command");
    if (get_ble_led_state() == LED_BLE_OFF) {
        // Avoid undefined behavior
        ERROR(comms, "Cannot start UWB: BLE has not been started");
    }
    if (get_uwb_led_state() == LED_UWB_ON) {
        ERROR(comms, "UWB is already on");
    }
    if (retrieveSetting(BELUGA_RANGE_EXTEND) == 1) {
        update_power_mode(POWER_MODE_HIGH);
    } else {
        update_power_mode(POWER_MODE_BYPASS);
    }
    k_sem_give(&k_sus_resp);
    k_sem_give(&k_sus_init);
    update_led_state(LED_UWB_ON);
    OK(comms);
}
AT_CMD_REGISTER(STARTUWB);

/**
 * The STOPUWB AT command
 *
 * This will stop UWB ranging
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(STOPUWB) {
    LOG_INF("Running STOPUWB command");
    if (get_uwb_led_state() == LED_UWB_OFF) {
        ERROR(comms, "UWB is not running");
    }
    k_sem_take(&k_sus_resp, K_FOREVER);
    k_sem_take(&k_sus_init, K_FOREVER);
    update_led_state(LED_UWB_OFF);
    OK(comms);
}
AT_CMD_REGISTER(STOPUWB);

/**
 * The STARTBLE AT command
 *
 * This will start BLE discovery
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(STARTBLE) {
    LOG_INF("Running STARTBLE) command");
    if (get_NODE_UUID() == 0) {
        ERROR(comms, "Cannot start BLE: Node ID is not set");
    } else if (get_ble_led_state() == LED_UWB_ON) {
        ERROR(comms, "BLE is already on");
    }
    k_sem_give(&print_list_sem);
    int err = enable_bluetooth();
    if (err) {
        k_sem_take(&print_list_sem, K_FOREVER);
        ERROR(comms, "Failed to start BLE (%d)", err);
    }
    update_led_state(LED_BLE_ON);
    OK(comms);
}
AT_CMD_REGISTER(STARTBLE);

/**
 * The STOPBLE AT command
 *
 * This will stop BLE discovery
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(STOPBLE) {
    LOG_INF("Running STOPBLE command");
    if (get_ble_led_state() == LED_UWB_OFF) {
        ERROR(comms, "BLE is already off");
    }
    int err = disable_bluetooth();
    if (err) {
        ERROR(comms, "Failed to stop BLE (%d)", err);
    }
    k_sem_take(&print_list_sem, K_FOREVER);
    update_led_state(LED_BLE_OFF);
    OK(comms);
}
AT_CMD_REGISTER(STOPBLE);

/**
 * The ID AT command
 *
 * This will set the ID of the Beluga node, or will print the current ID if the
 * ID argument is not present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(ID) {
    LOG_INF("Running ID command");
    READ_SETTING(comms, argc, 2, BELUGA_ID, "ID");
    int32_t newID;
    bool success = strtoint32(argv[1], &newID);

    if (!success || newID <= 0 || newID > (int32_t)UINT16_MAX) {
        ERROR(comms, "Invalid ID");
    }

    if (set_initiator_id((uint16_t)newID) != 0) {
        ERROR(comms, "Unable to set ID: UWB currently active");
    }

    // We know that UWB is inactive at this point
    set_responder_id((uint16_t)newID);
    update_node_id((uint16_t)newID);
    updateSetting(BELUGA_ID, newID);
    OK(comms);
}
AT_CMD_REGISTER(ID);

///**
// * The BOOTMODE AT command
// *
// * This will set the bootmode (determines if BLE and UWB is on at boot), or it
// * will print the current bootmode setting if the argument is not present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(BOOTMODE) {
//    LOG_INF("Running BOOTMODE command");
//    READ_SETTING(argc, 2, BELUGA_BOOTMODE, "Bootmode");
//    int32_t mode;
//    bool success = strtoint32(argv[1], &mode);
//
//    if (mode < 0 || mode > 2 || !success) {
//        ERROR("Invalid bootmode parameter");
//        return;
//    }
//
//    if (retrieveSetting(BELUGA_ID) <= 0) {
//        ERROR("Cannot set boot mode: Node has invalid ID");
//        return;
//    }
//
//    updateSetting(BELUGA_BOOTMODE, mode);
//    OK("Bootmode: %d", mode);
//}
//
///**
// * The RATE AT command
// *
// * This will set the polling rate of the UWB ranging, or it will get the
// current
// * polling rate of the ranging if the argument is not present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(RATE) {
//    LOG_INF("Running RATE command");
//    READ_SETTING(argc, 2, BELUGA_POLL_RATE, "Rate");
//    int32_t rate;
//    bool success = strtoint32(argv[1], &rate);
//
//    if (rate < 0 || rate > 500 || !success) {
//        ERROR("Invalid rate parameter");
//        return;
//    }
//
//    updateSetting(BELUGA_POLL_RATE, rate);
//    set_rate(rate);
//
//    // reconfig ble data
//    advertising_reconfig(rate != 0);
//    OK("Rate: %d", rate);
//}
//
///**
// * The CHANNEL AT command
// *
// * This will set the UWB channel, or it will get the current UWB channel if
// the
// * argument is not present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(CHANNEL) {
//    LOG_INF("Running CHANNEL command");
//    READ_SETTING(argc, 2, BELUGA_UWB_CHANNEL, "Channel");
//    int32_t channel;
//    int retVal;
//    bool success = strtoint32(argv[1], &channel);
//
//    if (!success) {
//        ERROR("Channel parameter input error");
//        return;
//    }
//
//    retVal = set_uwb_channel(channel);
//    if (retVal == -EBUSY) {
//        ERROR("Cannot set UWB parameter: UWB is active");
//        return;
//    } else if (retVal != 0) {
//        ERROR("Channel parameter input error");
//        return;
//    }
//
//    updateSetting(BELUGA_UWB_CHANNEL, channel);
//    OK();
//}
//
///**
// * The RESET AT command
// *
// * This will reset all the saved settings back to their defaults. For the
// reset
// * to take affect, the node must be rebooted.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(RESET) {
//    LOG_INF("Running RESET command");
//    resetBelugaSettings();
//    OK("Reset");
//}
//
///**
// * The TIMEOUT AT command
// *
// * Sets the amount of time that a node can stay within the neighbor list
// without
// * any updates, or it will get the current timeout if the argument is not
// * present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(TIMEOUT) {
//    LOG_INF("Running TIMEOUT command");
//    READ_SETTING(argc, 2, BELUGA_BLE_TIMEOUT, "Timeout");
//    int32_t timeout;
//    bool success = strtoint32(argv[1], &timeout);
//
//    if (!success || timeout < 0) {
//        ERROR("Invalid timeout value");
//        return;
//    }
//
//    updateSetting(BELUGA_BLE_TIMEOUT, timeout);
//    set_node_timeout(timeout);
//    OK();
//}
//
///**
// * The TXPOWER AT command
// *
// * Sets the TX power of the UWB to either the default or max setting, or it
// will
// * get the current TX power if the argument is not present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(TXPOWER) {
//    LOG_INF("Running TXPOWER command");
//    READ_SETTING(argc, 2, BELUGA_TX_POWER, "TX Power", print_tx_power);
//    int32_t arg1, coarse_control, fine_control;
//    bool value, success = strtoint32(argv[1], &arg1);
//    uint32_t power, mask = UINT8_MAX, new_setting;
//
//    switch (argc) {
//    case 2: {
//        if (success && int2bool(&value, arg1)) {
//            power = value ? TX_POWER_MAX : TX_POWER_MAN_DEFAULT;
//            set_tx_power(power);
//        } else {
//            ERROR("Tx power parameter input error");
//            return;
//        }
//        break;
//    }
//    case 3: {
//        ERROR("Invalid number of parameters");
//        return;
//    }
//    case 4:
//    default: {
//        if (!success || arg1 < 0 || arg1 > 3) {
//            ERROR("Invalid TX amplification stage");
//            return;
//        }
//        success = strtoint32(argv[2], &coarse_control);
//        if (!success || coarse_control < 0 || coarse_control > 7) {
//            ERROR("Invalid TX coarse gain");
//            return;
//        }
//        success = strtoint32(argv[3], &fine_control);
//        if (!success || fine_control < 0 || fine_control > 31) {
//            ERROR("Invalid TX fine gain");
//            return;
//        }
//        power = (uint32_t)retrieveSetting(BELUGA_TX_POWER);
//        coarse_control = (~coarse_control) & 0x7;
//        new_setting = (coarse_control << 5) | fine_control;
//        mask <<= 8 * arg1;
//        power &= ~mask;
//        power |= new_setting << 8 * arg1;
//        set_tx_power(power);
//        break;
//    }
//    }
//
//    updateSetting(BELUGA_TX_POWER, (int32_t)power);
//    OK();
//}
//
///**
// * The STREAMMODE AT command
// *
// * Sets the stream mode (print everything every 50 ms or only print updates),
// or
// * it will get the current stream mode if the argument is not present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(STREAMMODE) {
//    LOG_INF("Running STREAMMODE command");
//    READ_SETTING(argc, 2, BELUGA_STREAMMODE, "Stream Mode");
//    int32_t mode;
//    bool value, success = strtoint32(argv[1], &mode);
//
//    if (success && int2bool(&value, mode)) {
//        updateSetting(BELUGA_STREAMMODE, mode);
//        set_stream_mode(value);
//        OK();
//    } else {
//        ERROR("Stream mode parameter input error");
//    }
//}
//
///**
// * The TWRMODE AT command
// *
// * Sets the ranging mode of the UWB, or it will get the current ranging mode
// if
// * the argument is not present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(TWRMODE) {
//    LOG_INF("Running TWRMODE command");
//    READ_SETTING(argc, 2, BELUGA_TWR, "TWR");
//    int32_t twr;
//    bool value, success = strtoint32(argv[1], &twr);
//
//    if (success && int2bool(&value, twr)) {
//        updateSetting(BELUGA_TWR, twr);
//        set_twr_mode(value);
//        OK();
//    } else {
//        ERROR("TWR mode parameter input error");
//    }
//}
//
///**
// * The LEDMODE AT command
// *
// * Sets the LED mode, or it will get the current LED mode if the argument is
// not
// * present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(LEDMODE) {
//    LOG_INF("Running LEDMODE command");
//    READ_SETTING(argc, 2, BELUGA_LEDMODE, "LED Mode");
//    int32_t mode;
//    bool success = strtoint32(argv[1], &mode);
//
//    if (!success || mode < 0 || mode > 1) {
//        ERROR("LED mode parameter input error");
//        return;
//    }
//
//    updateSetting(BELUGA_LEDMODE, mode);
//    if (mode == 1) {
//        all_leds_off();
//    } else {
//        restore_led_states();
//    }
//
//    OK();
//}
//
///**
// * The REBOOT AT command
// *
// * Reboots the node.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(REBOOT) {
//    LOG_INF("Running REBOOT command");
//    OK();
//    printf("\r\n");
//    disable_bluetooth();
//    sys_reboot(SYS_REBOOT_COLD);
//}
//
///**
// * The PWRAMP AT command
// *
// * Enables/disables the external power amplifiers, or gets the current power
// * amplification setting if the argument is not present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(PWRAMP) {
//    LOG_INF("Running PWRAMP command");
//    READ_SETTING(argc, 2, BELUGA_RANGE_EXTEND, "Range Extension");
//    int32_t pwramp;
//    bool success = strtoint32(argv[1], &pwramp);
//    int err;
//
//    if (!success || pwramp < 0 || pwramp > 2) {
//        ERROR("Power amp parameter input error");
//        return;
//    }
//
//    if (pwramp == 0) {
//        err = update_power_mode(POWER_MODE_BYPASS);
//    } else if (pwramp == 1) {
//        err = update_power_mode(POWER_MODE_LOW);
//    } else {
//        err = update_power_mode(POWER_MODE_HIGH);
//    }
//
//    if (err == 0 || err == -ENODEV) {
//        if (pwramp == 0) {
//            update_led_state(LED_PWRAMP_OFF);
//        } else {
//            update_led_state(LED_PWRAMP_ON);
//        }
//
//        updateSetting(BELUGA_RANGE_EXTEND, pwramp);
//        if (err == -ENODEV) {
//            OK("BLE amplifier not supported. Only using UWB amplifier");
//        } else {
//            OK();
//        }
//    } else if (err == -EINVAL) {
//        ERROR("Power mode not recognized");
//    } else if (err == -ENOTSUP) {
//        ERROR("Not implemented");
//    } else {
//        ERROR("Power amplifier error occurred: %d", err);
//    }
//}
//
///**
// * The ANTENNA AT command
// *
// * Sets the active BLE antenna.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(ANTENNA) {
//    LOG_INF("Running ANTENNA command");
//    CHECK_ARGC(argc, 2);
//    int32_t antenna;
//    bool success = strtoint32(argv[1], &antenna);
//    int err;
//
//    if (!success || antenna < 1 || antenna > 2) {
//        ERROR("Antenna parameter input error");
//        return;
//    }
//
//    err = select_antenna(antenna);
//
//    if (err == 0) {
//        OK();
//    } else if (err == -EINVAL) {
//        ERROR("Invalid antenna selection");
//    } else if (err == -ENOTSUP) {
//        ERROR("Not implemented");
//    } else {
//        ERROR("Unknown error occurred: %d", err);
//    }
//}
//
///**
// * The TIME AT command
// *
// * Retrieves the current kernel timestamp.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(TIME) {
//    LOG_INF("Running TIME command");
//    OK("Time: %" PRId64, k_uptime_get());
//}
//
///**
// * The FORMAT AT command
// *
// * Sets the output format mode (0 for CSV mode, 1 for JSON mode), or gets the
// * current format mode if the argument is not present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(FORMAT) {
//    LOG_INF("Running FORMAT command");
//    READ_SETTING(argc, 2, BELUGA_OUT_FORMAT, "Format Mode",
//                 print_output_format);
//    int32_t mode;
//    bool success = strtoint32(argv[1], &mode);
//
//    if (!success || mode < 0 || mode > 1) {
//        ERROR("Format parameter input error");
//        return;
//    }
//
//    updateSetting(BELUGA_OUT_FORMAT, mode);
//    set_format_mode(mode == 1);
//    OK();
//}
//
///**
// * The DEEPSLEEP AT command
// *
// * Places the UWB and the BLE chips into deep sleep.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(DEEPSLEEP) {
//    LOG_INF("Running DEEPSLEEP command");
//    OK();
//    printf("\r\n");
//    enter_deep_sleep();
//}
//
///**
// * The PHR AT command
// *
// * Sets the UWB PHR mode, or it will retrieve the current PHR mode if the
// * argument is not present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(PHR) {
//    LOG_INF("Running PHR command");
//    READ_SETTING(argc, 2, BELUGA_UWB_PHR, "UWB PHR mode");
//    int32_t phr;
//    int retVal;
//    bool success = strtoint32(argv[1], &phr);
//
//    if (!success) {
//        ERROR("PHR mode parameter input error");
//        return;
//    }
//
//    retVal = uwb_set_phr_mode((enum uwb_phr_mode)phr);
//    if (retVal == -EBUSY) {
//        ERROR("Cannot set UWB parameter: UWB is active");
//        return;
//    } else if (retVal != 0) {
//        ERROR("PHR mode parameter input error");
//        return;
//    }
//
//    updateSetting(BELUGA_UWB_PHR, phr);
//    OK();
//}
//
///**
// * The DATARATE AT command
// *
// * Sets the UWB data rate, or gets the current data rate if the argument is
// not
// * present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(DATARATE) {
//    LOG_INF("Running DATARATE command");
//    READ_SETTING(argc, 2, BELUGA_UWB_DATA_RATE, "UWB data rate",
//                 print_uwb_datarate);
//    int32_t rate;
//    int retVal;
//    bool success = strtoint32(argv[1], &rate);
//
//    if (!success) {
//        ERROR("Data rate parameter input error");
//        return;
//    }
//
//    retVal = uwb_set_datarate((enum uwb_datarate)rate);
//    if (retVal == -EBUSY) {
//        ERROR("Cannot set UWB parameter: UWB is active");
//        return;
//    } else if (retVal != 0) {
//        ERROR("Data rate parameter input error");
//        return;
//    }
//
//    updateSetting(BELUGA_UWB_DATA_RATE, rate);
//    OK();
//}
//
///**
// * The PULSERATE AT command
// *
// * Sets the UWB pulse rate, or gets the current pulse rate if the argument is
// * not present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(PULSERATE) {
//    LOG_INF("Running PULSERATE command");
//    READ_SETTING(argc, 2, BELUGA_UWB_PULSE_RATE, "Pulse Rate",
//                 print_pulse_rate);
//    int32_t rate;
//    int retVal;
//    bool success = strtoint32(argv[1], &rate);
//
//    if (!success) {
//        ERROR("Invalid pulse rate input parameter");
//        return;
//    }
//
//    retVal = uwb_set_pulse_rate((enum uwb_pulse_rate)rate);
//    if (retVal == -EBUSY) {
//        ERROR("Cannot set UWB parameter: UWB is active");
//        return;
//    } else if (retVal != 0) {
//        ERROR("Pulse rate parameter input error");
//        return;
//    }
//
//    updateSetting(BELUGA_UWB_PULSE_RATE, rate);
//    OK();
//}
//
///**
// * The PREAMBLE AT command
// *
// * Sets the UWB Preamble length, or gets the current preamble length if the
// * argument is not present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(PREAMBLE) {
//    LOG_INF("Running PREAMBLE command");
//    READ_SETTING(argc, 2, BELUGA_UWB_PREAMBLE, "Preamble length");
//    int32_t preamble;
//    int retVal;
//    bool success = strtoint32(argv[1], &preamble);
//
//    if (!success) {
//        ERROR("Invalid Preamble length setting");
//        return;
//    }
//
//    retVal = uwb_set_preamble((enum uwb_preamble_length)preamble);
//    if (retVal == -EBUSY) {
//        ERROR("Cannot set UWB parameter: UWB is active");
//        return;
//    } else if (retVal != 0) {
//        ERROR("Preamble parameter input error");
//        return;
//    }
//
//    updateSetting(BELUGA_UWB_PREAMBLE, preamble);
//    OK();
//}
//
///**
// * The PAC AT command
// *
// * Sets the UWB PAC size, it it gets the current PAC size if the argument is
// not
// * present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(PAC) {
//    LOG_INF("Running PAC command");
//    READ_SETTING(argc, 2, BELUGA_UWB_PAC, "PAC Size", print_pac_size);
//    int32_t pac_size;
//    int retVal;
//    bool success = strtoint32(argv[1], &pac_size);
//
//    if (!success) {
//        ERROR("Invalid PAC size setting");
//        return;
//    }
//
//    retVal = set_pac_size((enum uwb_pac)pac_size);
//    if (retVal == -EBUSY) {
//        ERROR("Cannot set UWB parameter: UWB is active");
//        return;
//    } else if (retVal != 0) {
//        ERROR("PAC Size parameter input error");
//        return;
//    }
//
//    updateSetting(BELUGA_UWB_PAC, pac_size);
//    OK();
//}
//
///**
// * The SFD AT command
// *
// * Sets the SFD to be used (0 for standard, 1 for non-standard), or gets the
// * current SFD setting if the argument is not present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(SFD) {
//    LOG_INF("Running SFD command");
//    READ_SETTING(argc, 2, BELUGA_UWB_NSSFD, "Nonstandard SFD");
//    int32_t sfd;
//    int retVal;
//    bool success = strtoint32(argv[1], &sfd);
//
//    if (!success) {
//        ERROR("Invalid Preamble length setting");
//        return;
//    }
//
//    retVal = set_sfd_mode((enum uwb_sfd)sfd);
//    if (retVal == -EBUSY) {
//        ERROR("Cannot set UWB parameter: UWB is active");
//        return;
//    } else if (retVal != 0) {
//        ERROR("SFD parameter input error");
//        return;
//    }
//
//    updateSetting(BELUGA_UWB_NSSFD, sfd);
//    OK();
//}
//
///**
// * The PANID AT command
// *
// * Sets the UWB PAN ID, or gets the current PAN ID if the argument is not
// * present.
// *
// * @param[in] argc The argument count
// * @param[in] argv The parsed arguments
// */
// AT_CMD_DEFINE(PANID) {
//    LOG_INF("Running PANID command");
//    READ_SETTING(argc, 2, BELUGA_PAN_ID, "PAN ID", print_pan_id);
//    int32_t pan_id;
//    int retVal;
//    bool success = strtoint32(argv[1], &pan_id);
//
//    if (!success || pan_id < INT32_C(0) || pan_id > (uint32_t)UINT16_MAX) {
//        ERROR("Invalid PAN ID");
//        return;
//    }
//
//    retVal = set_initiator_pan_id((uint16_t)pan_id);
//
//    if (retVal != 0) {
//        ERROR("Cannot set PAN ID: UWB Active");
//        return;
//    }
//    set_responder_pan_id((uint16_t)pan_id);
//    updateSetting(BELUGA_PAN_ID, pan_id);
//    OK();
//}
