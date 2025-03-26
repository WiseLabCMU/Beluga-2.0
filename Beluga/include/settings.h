/**
 * @file settings.c
 * @brief Beluga settings management module.
 *
 * Contains functions and structures related to managing settings
 * for the Beluga. It includes functionality for reading, writing,
 * resetting settings. The settings are stored as key-value
 * pairs and can be persisted using a storage backend. This module handles
 * both the initialization of settings from persistent storage and the
 * runtime handling of settings updates.
 *
 * @author Tom Schmitz
 * @date 7/11/24
 */

#ifndef BELUGA_SETTINGS_H
#define BELUGA_SETTINGS_H

#include <deca_regs.h>
#include <stdint.h>

/**
 * The settings for Beluga and their default values
 *
 * - Node ID
 * - Boot mode
 * - Rate that UWB initiates ranging requests
 * - UWB channel used
 * - Timeout before removing from neighbor list
 * - UWB transmission power
 * - Neighbor list streaming option
 * - UWB ranging mode
 * - LED mode
 * - Output format
 * - Power amplifier state
 * - UWB PHR mode
 * - UWB data rate
 * - UWB pulse repetition rate
 * - UWB preamble length
 * - UWB PAC size
 * - UWB SFD length option
 * - UWB PAN ID
 * .
 *
 * Format: setting name, default value
 */
#define FOREACH_BELUGA_SETTING(FUNC)                                           \
    FUNC(ID, 0)                                                                \
    FUNC(BOOTMODE, 0)                                                          \
    FUNC(POLL_RATE, 250)                                                       \
    FUNC(UWB_CHANNEL, 5)                                                       \
    FUNC(BLE_TIMEOUT, 9000)                                                    \
    FUNC(TX_POWER, TX_POWER_MAN_DEFAULT)                                       \
    FUNC(STREAMMODE, 0)                                                        \
    FUNC(TWR, 1)                                                               \
    FUNC(LEDMODE, 0)                                                           \
    FUNC(OUT_FORMAT, 0)                                                        \
    FUNC(RANGE_EXTEND, 0)                                                      \
    FUNC(UWB_PHR, 0)                                                           \
    FUNC(UWB_DATA_RATE, 0)                                                     \
    FUNC(UWB_PULSE_RATE, 1)                                                    \
    FUNC(UWB_PREAMBLE, 128)                                                    \
    FUNC(UWB_PAC, 0)                                                           \
    FUNC(UWB_NSSFD, 0)                                                         \
    FUNC(PAN_ID, 0xDECA)                                                       \
    FUNC(EVICTION_SCHEME, 1)                                                   \
    FUNC(VERBOSE, 0)

/**
 * Helper for generating the enumerators for beluga settings
 */
#define GENERATE_ENUM(setting_, ...) BELUGA_##setting_,

/**
 * @brief Beluga settings.
 *
 * Represents various configuration settings for Beluga.
 * Each setting is associated with a specific configuration parameter, such as
 * UWB channel, LED mode, transmission power, and so on. These settings are used
 * in the system to configure or modify different device behaviors.
 */
enum beluga_setting {
    FOREACH_BELUGA_SETTING(GENERATE_ENUM)
        BELUGA_RESERVED, ///< Reserved settings enumerator indicating the last
                         ///< setting
};

#undef GENERATE_ENUM

/**
 * Default (invalid) value for the ID setting
 */
#define DEFAULT_ID_SETTING INT32_C(0)

/**
 * Default (invalid) value for all of the settings (except for the ID setting)
 */
#define DEFAULT_SETTING INT32_C(-1)

/**
 * @brief Write a new value for a beluga setting
 * @param[in] setting The beluga setting to update
 * @param[in] value The new value of the setting
 */
void updateSetting(enum beluga_setting setting, int32_t value);

/**
 * @brief Gets a current beluga setting
 *
 * @param[in] setting The beluga setting to retrieve
 *
 * @return The value of the setting
 * @return -1 if setting is invalid
 */
int32_t retrieveSetting(enum beluga_setting setting);

/**
 * @brief Erases/resets all the beluga settings to their default values
 */
void resetBelugaSettings(void);

/**
 * @brief Initializes the settings subsystem and loads all the settings from
 * persistent storage
 * @return 0 upon success
 * @return negative error code otherwise
 */
int initBelugaSettings(void);

#endif // BELUGA_SETTINGS_H
