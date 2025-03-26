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

#include <deca_types.h>
#include <initiator.h>
#include <responder.h>

#include <ble_app.h>
#include <ctype.h>
#include <errno.h>

#include <app_leds.h>
#include <list_monitor.h>
#include <list_neighbors.h>
#include <power_manager.h>
#include <range_extension.h>
#include <ranging.h>
#include <serial/comms.h>
#include <settings.h>
#include <stdlib.h>
#include <utils.h>
#include <zephyr/logging/log.h>

/**
 * Logger for the AT commands
 */
LOG_MODULE_REGISTER(at_commands, CONFIG_AT_COMMANDS_LOG_LEVEL);

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
#define READ_SETTING(_comms, argc, required, setting, settingstr)              \
    do {                                                                       \
        if ((argc) < (required)) {                                             \
            int32_t _setting = retrieveSetting(setting);                       \
            OK(_comms, settingstr ": %" PRIu32, (uint32_t)_setting);           \
        }                                                                      \
    } while (0)

/**
 * @brief Prints out the saved setting in hex format if the minimum amount of
 * tokens have not been passed in
 *
 * @param[in] argc The amount of tokens parsed
 * @param[in] required The amount of tokens required to do something other than
 * "read the setting"
 * @param[in] setting The enum that defines the setting
 * @param[in] settingstr The string representation of the setting
 * @param[in] callback An optional custom print function for the setting
 * @param[in] padding Format string padding for the hex number. For example,
 * if it is desired to show at least 8 digits with leading zeros, this would
 * be "08". If no zero padding is wanted, then use an empty string ("").
 */
#define READ_SETTING_HEX(_comms, argc, required, setting, settingstr, padding) \
    do {                                                                       \
        if ((argc) < (required)) {                                             \
            int32_t _setting = retrieveSetting(setting);                       \
            OK(_comms, settingstr ": 0x%" padding PRIX32, (uint32_t)_setting); \
        }                                                                      \
    } while (0)

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
    enum power_mode pwramp =
        (enum power_mode)retrieveSetting(BELUGA_RANGE_EXTEND);
    update_power_mode(pwramp);
    k_sem_give(&k_sus_resp);
    k_sem_give(&k_sus_init);
    update_led_state(LED_UWB_ON);
    AT_OK(comms, "Started UWB");
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
    AT_OK(comms, "Stopped UWB");
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
    } else if (get_ble_led_state() == LED_BLE_ON) {
        ERROR(comms, "BLE is already on");
    }
    k_sem_give(&print_list_sem);
    int err = enable_bluetooth();
    if (err) {
        k_sem_take(&print_list_sem, K_FOREVER);
        ERROR(comms, "Failed to start BLE (%d)", err);
    }
    update_led_state(LED_BLE_ON);
    AT_OK(comms, "Started BLE");
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
    if (get_ble_led_state() == LED_BLE_OFF) {
        ERROR(comms, "BLE is already off");
    }
    int err = disable_bluetooth();
    if (err) {
        ERROR(comms, "Failed to stop BLE (%d)", err);
    }
    k_sem_take(&print_list_sem, K_FOREVER);
    update_led_state(LED_BLE_OFF);
    AT_OK(comms, "Stopped BLE");
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
    AT_OK(comms, "ID: %d", newID);
}
AT_CMD_REGISTER(ID);

/**
 * The BOOTMODE AT command
 *
 * This will set the bootmode (determines if BLE and UWB is on at boot), or it
 * will print the current bootmode setting if the argument is not present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(BOOTMODE) {
    LOG_INF("Running BOOTMODE command");
    READ_SETTING(comms, argc, 2, BELUGA_BOOTMODE, "Bootmode");
    int32_t mode;
    bool success = strtoint32(argv[1], &mode);

    if (mode < 0 || mode > 2 || !success) {
        ERROR(comms, "Invalid bootmode parameter");
    }

    if (retrieveSetting(BELUGA_ID) <= 0) {
        ERROR(comms, "Cannot set boot mode: Node has invalid ID");
    }

    updateSetting(BELUGA_BOOTMODE, mode);
    AT_OK(comms, "Bootmode: %d", mode);
}
AT_CMD_REGISTER(BOOTMODE);

/**
 * The RATE AT command
 *
 * This will set the polling rate of the UWB ranging, or it will get the
 current
 * polling rate of the ranging if the argument is not present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(RATE) {
    LOG_INF("Running RATE command");
    READ_SETTING(comms, argc, 2, BELUGA_POLL_RATE, "Rate");
    int32_t rate;
    bool success = strtoint32(argv[1], &rate);

    if (rate < 0 || rate > (int32_t)CONFIG_MAX_POLLING_RATE || !success) {
        ERROR(comms, "Invalid rate parameter");
    }

    updateSetting(BELUGA_POLL_RATE, rate);
    set_rate(rate);

    // reconfig ble data
    advertising_reconfig(rate != 0);
    AT_OK(comms, "Rate: %d", rate);
}
AT_CMD_REGISTER(RATE);

/**
 * The CHANNEL AT command
 *
 * This will set the UWB channel, or it will get the current UWB channel if
 the
 * argument is not present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(CHANNEL) {
    LOG_INF("Running CHANNEL command");
    READ_SETTING(comms, argc, 2, BELUGA_UWB_CHANNEL, "Channel");
    int32_t channel, pwramp;
    int retVal;
    bool success = strtoint32(argv[1], &channel);

    if (!success) {
        ERROR(comms, "Channel parameter input error");
    }

    pwramp = retrieveSetting(BELUGA_RANGE_EXTEND);
    if (IS_UWB_AMP_ON(pwramp) && !UWB_AMP_CHANNEL(channel)) {
        ERROR(comms, "UWB power amplifier is currently active and channels 2, "
                     "3, and 4 can only be used with the amplifier");
    }

    retVal = set_uwb_channel(channel);
    if (retVal == -EBUSY) {
        ERROR(comms, "Cannot set UWB parameter: UWB is active");
    } else if (retVal != 0) {
        ERROR(comms, "Channel parameter input error");
    }

    updateSetting(BELUGA_UWB_CHANNEL, channel);
    AT_OK(comms, "Channel: %d", channel);
}
AT_CMD_REGISTER(CHANNEL);

/**
 * The RESET AT command
 *
 * This will reset all the saved settings back to their defaults. For the
 reset
 * to take affect, the node must be rebooted.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(RESET) {
    LOG_INF("Running RESET command");
    resetBelugaSettings();
    OK(comms, "Reset");
}
AT_CMD_REGISTER(RESET);

/**
 * The TIMEOUT AT command
 *
 * Sets the amount of time that a node can stay within the neighbor list
 without
 * any updates, or it will get the current timeout if the argument is not
 * present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(TIMEOUT) {
    LOG_INF("Running TIMEOUT command");
    READ_SETTING(comms, argc, 2, BELUGA_BLE_TIMEOUT, "Timeout");
    int32_t timeout;
    bool success = strtoint32(argv[1], &timeout);

    if (!success || timeout < 0) {
        ERROR(comms, "Invalid timeout value");
    }

    updateSetting(BELUGA_BLE_TIMEOUT, timeout);
    set_node_timeout(timeout);
    AT_OK(comms, "Timeout: %d", timeout);
}
AT_CMD_REGISTER(TIMEOUT);

/**
 * The TXPOWER AT command
 *
 * Sets the TX power of the UWB to either the default or max setting, or it
 will
 * get the current TX power if the argument is not present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(TXPOWER) {
    LOG_INF("Running TXPOWER command");
    READ_SETTING_HEX(comms, argc, 2, BELUGA_TX_POWER, "TX Power", "08");
    int32_t arg1, coarse_control, fine_control;
    bool value, success = strtoint32(argv[1], &arg1);
    uint32_t power, mask = UINT8_MAX, new_setting;

    switch (argc) {
    case 2: {
        if (success && int2bool(&value, arg1)) {
            power = value ? TX_POWER_MAX : TX_POWER_MAN_DEFAULT;
            set_tx_power(power);
        } else {
            ERROR(comms, "Tx power parameter input error");
        }
        break;
    }
    case 3: {
        ERROR(comms, "Invalid number of parameters");
    }
    case 4:
    default: {
        if (!success || arg1 < 0 || arg1 > 3) {
            ERROR(comms, "Invalid TX amplification stage");
        }
        success = strtoint32(argv[2], &coarse_control);
        if (!success || coarse_control < 0 || coarse_control > 7) {
            ERROR(comms, "Invalid TX coarse gain");
        }
        success = strtoint32(argv[3], &fine_control);
        if (!success || fine_control < 0 || fine_control > 31) {
            ERROR(comms, "Invalid TX fine gain");
        }
        power = (uint32_t)retrieveSetting(BELUGA_TX_POWER);
        coarse_control = (~coarse_control) & 0x7;
        new_setting = (coarse_control << 5) | fine_control;
        mask <<= 8 * arg1;
        power &= ~mask;
        power |= new_setting << 8 * arg1;
        set_tx_power(power);
        break;
    }
    }

    updateSetting(BELUGA_TX_POWER, (int32_t)power);
    AT_OK(comms, "TX Power: 0x%08X", power);
}
AT_CMD_REGISTER(TXPOWER);

/**
 * The STREAMMODE AT command
 *
 * Sets the stream mode (print everything every 50 ms or only print updates),
 or
 * it will get the current stream mode if the argument is not present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(STREAMMODE) {
    LOG_INF("Running STREAMMODE command");
    READ_SETTING(comms, argc, 2, BELUGA_STREAMMODE, "Stream Mode");
    int32_t mode;
    bool value, success = strtoint32(argv[1], &mode);

    if (success && int2bool(&value, mode)) {
        updateSetting(BELUGA_STREAMMODE, mode);
        set_stream_mode(value);
        OK(comms, "Stream mode: %d", mode);
    }
    ERROR(comms, "Stream mode parameter input error");
}
AT_CMD_REGISTER(STREAMMODE);

/**
 * The TWRMODE AT command
 *
 * Sets the ranging mode of the UWB, or it will get the current ranging mode
 if
 * the argument is not present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(TWRMODE) {
    LOG_INF("Running TWRMODE command");
    READ_SETTING(comms, argc, 2, BELUGA_TWR, "TWR");
    int32_t twr;
    bool value, success = strtoint32(argv[1], &twr);

    if (success && int2bool(&value, twr)) {
        updateSetting(BELUGA_TWR, twr);
        set_twr_mode(value);
        AT_OK(comms, "TWR: %d", twr);
    }
    ERROR(comms, "TWR mode parameter input error");
}
AT_CMD_REGISTER(TWRMODE);

/**
 * The LEDMODE AT command
 *
 * Sets the LED mode, or it will get the current LED mode if the argument is
 not
 * present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(LEDMODE) {
    LOG_INF("Running LEDMODE command");
    READ_SETTING(comms, argc, 2, BELUGA_LEDMODE, "LED Mode");
    int32_t mode;
    bool success = strtoint32(argv[1], &mode);

    if (!success || mode < 0 || mode > 1) {
        ERROR(comms, "LED mode parameter input error");
    }

    updateSetting(BELUGA_LEDMODE, mode);
    if (mode == 1) {
        all_leds_off();
    } else {
        restore_led_states();
    }

    AT_OK(comms, "LED mode: %d", mode);
}
AT_CMD_REGISTER(LEDMODE);

/**
 * The REBOOT AT command
 *
 * Reboots the node.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(REBOOT) {
    LOG_INF("Running REBOOT command");
    AT_OK_NOW(comms, "Rebooting");
    disable_bluetooth();
    sys_reboot(SYS_REBOOT_COLD);
}
AT_CMD_REGISTER(REBOOT);

/**
 * The PWRAMP AT command
 *
 * Enables/disables the external power amplifiers, or gets the current power
 * amplification setting if the argument is not present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(PWRAMP) {
    LOG_INF("Running PWRAMP command");
    READ_SETTING(comms, argc, 2, BELUGA_RANGE_EXTEND, "Range Extension");
    int32_t pwramp;
    bool success = strtoint32(argv[1], &pwramp);
    int err;

    if (!success || pwramp < 0 || pwramp > 5) {
        ERROR(comms, "Power amp parameter input error");
    }

    int32_t channel = retrieveSetting(BELUGA_UWB_CHANNEL);

    switch (pwramp) {
    case 0: {
        err = update_power_mode(POWER_MODE_EXTERNAL_AMPS_OFF);
        break;
    }
    case 1: {
        if (UWB_AMP_CHANNEL(channel)) {
            err = update_power_mode(POWER_MODE_BYPASS);
        } else {
            err = -EFAULT;
        }
        break;
    }
    case 2: {
        err = update_power_mode(POWER_MODE_LOW_NO_UWB);
        break;
    }
    case 3: {
        if (UWB_AMP_CHANNEL(channel)) {
            err = update_power_mode(POWER_MODE_LOW);
        } else {
            err = -EFAULT;
        }
        break;
    }
    case 4: {
        err = update_power_mode(POWER_MODE_HIGH_NO_UWB);
        break;
    }
    case 5: {
        if (UWB_AMP_CHANNEL(channel)) {
            err = update_power_mode(POWER_MODE_HIGH);
        } else {
            err = -EFAULT;
        }
        break;
    }
    default: {
        ERROR(comms, "Power amp parameter input error");
        break;
    }
    }

    if (err == 0 || err == -ENODEV) {
        if (pwramp == 0) {
            update_led_state(LED_PWRAMP_OFF);
        } else {
            update_led_state(LED_PWRAMP_ON);
        }

        updateSetting(BELUGA_RANGE_EXTEND, pwramp);
        if (err == -ENODEV) {
            at_msg(comms,
                   "BLE amplifier not supported. Only using UWB amplifier ");
        }
        AT_OK(comms, "Range Extension: %d", pwramp);
    } else if (err == -EINVAL) {
        ERROR(comms, "Power mode not recognized");
    } else if (err == -ENOTSUP) {
        ERROR(comms, "Not implemented");
    } else if (err == -EFAULT) {
        ERROR(comms,
              "Cannot turn on UWB amplifier. Must be on channel 1 or 2 "
              "(currently on channel %" PRId32,
              channel);
    }
    ERROR(comms, "Power amplifier error occurred: %d", err);
}
AT_CMD_COND_REGISTER(IS_ENABLED(CONFIG_BELUGA_RANGE_EXTENSION), PWRAMP);

/**
 * The ANTENNA AT command
 *
 * Sets the active BLE antenna.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(ANTENNA) {
    LOG_INF("Running ANTENNA command");
    CHECK_ARGC(comms, argc, 2);
    int32_t antenna;
    bool success = strtoint32(argv[1], &antenna);
    int err;

    if (!success || antenna < 1 || antenna > 2) {
        ERROR(comms, "Antenna parameter input error");
    }

    err = select_antenna(antenna);

    if (err == 0) {
        AT_OK(comms, "Antenna: %d", antenna);
    } else if (err == -EINVAL) {
        ERROR(comms, "Invalid antenna selection");
    } else if (err == -ENOTSUP) {
        ERROR(comms, "Not implemented");
    } else {
        ERROR(comms, "Unknown error occurred: %d", err);
    }
}
AT_CMD_COND_REGISTER(IS_ENABLED(CONFIG_BELUGA_RANGE_EXTENSION), ANTENNA);

/**
 * The TIME AT command
 *
 * Retrieves the current kernel timestamp.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(TIME) {
    LOG_INF("Running TIME command");
    OK(comms, "Time: %" PRId64, k_uptime_get());
}
AT_CMD_REGISTER(TIME);

/**
 * The FORMAT AT command
 *
 * Sets the output format mode (0 for CSV mode, 1 for JSON mode), or gets the
 * current format mode if the argument is not present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(FORMAT) {
    LOG_INF("Running FORMAT command");
    READ_SETTING(comms, argc, 2, BELUGA_OUT_FORMAT, "Format Mode");
    int32_t mode;
    bool success = strtoint32(argv[1], &mode);

    if (!success || mode < 0 || mode > 2) {
        ERROR(comms, "Format parameter input error");
    }

    updateSetting(BELUGA_OUT_FORMAT, mode);
    set_format(comms, (enum comms_out_format_mode)mode);
    AT_OK(comms, "Format Mode: %d", mode);
}
AT_CMD_REGISTER(FORMAT);

/**
 * The DEEPSLEEP AT command
 *
 * Places the UWB and the BLE chips into deep sleep.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(DEEPSLEEP) {
    LOG_INF("Running DEEPSLEEP command");
    AT_OK_NOW(comms, "Entering into deep sleep");
    enter_deep_sleep();
}
AT_CMD_REGISTER(DEEPSLEEP);

/**
 * The PHR AT command
 *
 * Sets the UWB PHR mode, or it will retrieve the current PHR mode if the
 * argument is not present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(PHR) {
    LOG_INF("Running PHR command");
    READ_SETTING(comms, argc, 2, BELUGA_UWB_PHR, "UWB PHR mode");
    int32_t phr;
    int retVal;
    bool success = strtoint32(argv[1], &phr);

    if (!success) {
        ERROR(comms, "PHR mode parameter input error");
    }

    retVal = uwb_set_phr_mode((enum uwb_phr_mode)phr);
    if (retVal == -EBUSY) {
        ERROR(comms, "Cannot set UWB parameter: UWB is active");
    } else if (retVal != 0) {
        ERROR(comms, "PHR mode parameter input error");
    }

    updateSetting(BELUGA_UWB_PHR, phr);
    AT_OK(comms, "UWB PHR mode %d", phr);
}
AT_CMD_REGISTER(PHR);

/**
 * The DATARATE AT command
 *
 * Sets the UWB data rate, or gets the current data rate if the argument is
 not
 * present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(DATARATE) {
    LOG_INF("Running DATARATE command");
    READ_SETTING(comms, argc, 2, BELUGA_UWB_DATA_RATE, "UWB data rate");
    int32_t rate;
    int retVal;
    bool success = strtoint32(argv[1], &rate);

    if (!success) {
        ERROR(comms, "Data rate parameter input error");
    }

    retVal = uwb_set_datarate((enum uwb_datarate)rate);
    if (retVal == -EBUSY) {
        ERROR(comms, "Cannot set UWB parameter: UWB is active");
    } else if (retVal != 0) {
        ERROR(comms, "Data rate parameter input error");
    }

    updateSetting(BELUGA_UWB_DATA_RATE, rate);
    AT_OK(comms, "UWB data rate %d", rate);
}
AT_CMD_REGISTER(DATARATE);

/**
 * The PULSERATE AT command
 *
 * Sets the UWB pulse rate, or gets the current pulse rate if the argument is
 * not present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(PULSERATE) {
    LOG_INF("Running PULSERATE command");
    READ_SETTING(comms, argc, 2, BELUGA_UWB_PULSE_RATE, "Pulse Rate");
    int32_t rate;
    int retVal;
    bool success = strtoint32(argv[1], &rate);

    if (!success) {
        ERROR(comms, "Invalid pulse rate input parameter");
    }

    retVal = uwb_set_pulse_rate((enum uwb_pulse_rate)rate);
    if (retVal == -EBUSY) {
        ERROR(comms, "Cannot set UWB parameter: UWB is active");
    } else if (retVal != 0) {
        ERROR(comms, "Pulse rate parameter input error");
    }

    updateSetting(BELUGA_UWB_PULSE_RATE, rate);
    AT_OK(comms, "Pulse rate: %d", rate);
}
AT_CMD_REGISTER(PULSERATE);

/**
 * The PREAMBLE AT command
 *
 * Sets the UWB Preamble length, or gets the current preamble length if the
 * argument is not present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(PREAMBLE) {
    LOG_INF("Running PREAMBLE command");
    READ_SETTING(comms, argc, 2, BELUGA_UWB_PREAMBLE, "Preamble length");
    int32_t preamble;
    int retVal;
    bool success = strtoint32(argv[1], &preamble);

    if (!success) {
        ERROR(comms, "Invalid Preamble length setting");
    }

    retVal = uwb_set_preamble((enum uwb_preamble_length)preamble);
    if (retVal == -EBUSY) {
        ERROR(comms, "Cannot set UWB parameter: UWB is active");
    } else if (retVal != 0) {
        ERROR(comms, "Preamble parameter input error");
    }

    updateSetting(BELUGA_UWB_PREAMBLE, preamble);
    AT_OK(comms, "Preamble length: %d", preamble);
}
AT_CMD_REGISTER(PREAMBLE);

/**
 * The PAC AT command
 *
 * Sets the UWB PAC size, it gets the current PAC size if the argument is
 not
 * present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(PAC) {
    LOG_INF("Running PAC command");
    READ_SETTING(comms, argc, 2, BELUGA_UWB_PAC, "PAC Size");
    int32_t pac_size;
    int retVal;
    bool success = strtoint32(argv[1], &pac_size);

    if (!success) {
        ERROR(comms, "Invalid PAC size setting");
    }

    retVal = set_pac_size((enum uwb_pac)pac_size);
    if (retVal == -EBUSY) {
        ERROR(comms, "Cannot set UWB parameter: UWB is active");
    } else if (retVal != 0) {
        ERROR(comms, "PAC Size parameter input error");
    }

    updateSetting(BELUGA_UWB_PAC, pac_size);
    AT_OK(comms, "PAC size: %d", pac_size);
}
AT_CMD_REGISTER(PAC);

/**
 * The SFD AT command
 *
 * Sets the SFD to be used (0 for standard, 1 for non-standard), or gets the
 * current SFD setting if the argument is not present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(SFD) {
    LOG_INF("Running SFD command");
    READ_SETTING(comms, argc, 2, BELUGA_UWB_NSSFD, "Nonstandard SFD");
    int32_t sfd;
    int retVal;
    bool success = strtoint32(argv[1], &sfd);

    if (!success) {
        ERROR(comms, "Invalid Preamble length setting");
    }

    retVal = set_sfd_mode((enum uwb_sfd)sfd);
    if (retVal == -EBUSY) {
        ERROR(comms, "Cannot set UWB parameter: UWB is active");
    } else if (retVal != 0) {
        ERROR(comms, "SFD parameter input error");
    }

    updateSetting(BELUGA_UWB_NSSFD, sfd);
    AT_OK(comms, "Nonstandard SFD: %d", sfd);
}
AT_CMD_REGISTER(SFD);

/**
 * The PANID AT command
 *
 * Sets the UWB PAN ID, or gets the current PAN ID if the argument is not
 * present.
 *
 * @param[in] argc The argument count
 * @param[in] argv The parsed arguments
 */
AT_CMD_DEFINE(PANID) {
    LOG_INF("Running PANID command");
    READ_SETTING_HEX(comms, argc, 2, BELUGA_PAN_ID, "PAN ID", "04");
    int32_t pan_id;
    int retVal;
    bool success = strtoint32(argv[1], &pan_id);

    if (!success || pan_id < INT32_C(0) || pan_id > (uint32_t)UINT16_MAX) {
        ERROR(comms, "Invalid PAN ID");
    }

    retVal = set_initiator_pan_id((uint16_t)pan_id);

    if (retVal != 0) {
        ERROR(comms, "Cannot set PAN ID: UWB Active");
    }
    set_responder_pan_id((uint16_t)pan_id);
    updateSetting(BELUGA_PAN_ID, pan_id);
    AT_OK(comms, "PAN ID: 0x%04X", (uint16_t)pan_id);
}
AT_CMD_REGISTER(PANID);

/**
 * The EVICT AT command
 * @param[in] comms Pointer to the comms instance
 * @param[in] argc Number of arguments
 * @param[in] argv The arguments
 * @return 0 upon success
 * @return 1 upon error
 */
AT_CMD_DEFINE(EVICT) {
    LOG_INF("Running EVICTION command");
    READ_SETTING(comms, argc, 2, BELUGA_EVICTION_SCHEME, "Eviction Scheme");
    int32_t scheme;
    bool success = strtoint32(argv[1], &scheme);

    if (!success || scheme < INT32_C(0) ||
        scheme >= (int32_t)EVICT_POLICY_INVALID) {
        ERROR(comms, "Invalid eviction scheme");
    }

    set_node_eviction_policy(scheme);
    updateSetting(BELUGA_EVICTION_SCHEME, scheme);
    AT_OK(comms, "Eviction scheme: %d", scheme);
}
AT_CMD_COND_REGISTER(IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT), EVICT);

/**
 * Sets the verbosity for the commands
 * @param[in] comms Pointer to the comms instance
 * @param[in] argc Number of arguments
 * @param[in] argv The arguments
 * @return 0 upon success
 * @return 1 upon error
 */
AT_CMD_DEFINE(VERBOSE) {
    LOG_INF("Running VERBOSE command");
    READ_SETTING(comms, argc, 2, BELUGA_VERBOSE, "Verbose");
    int32_t mode;
    bool success = strtoint32(argv[1], &mode);

    if (!success || mode < 0 || mode > 1) {
        ERROR(comms, "Invalid verbose mode");
    }

    set_verbosity(comms, mode == 1);
    updateSetting(BELUGA_VERBOSE, mode);

    AT_OK(comms, "Verbose: %d", mode);
}
AT_CMD_REGISTER(VERBOSE);
