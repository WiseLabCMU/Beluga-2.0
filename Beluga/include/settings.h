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

#include <stdint.h>

/**
 * @brief Beluga settings.
 *
 * Represents various configuration settings for Beluga.
 * Each setting is associated with a specific configuration parameter, such as
 * UWB channel, LED mode, transmission power, and so on. These settings are used
 * in the system to configure or modify different device behaviors.
 */
enum beluga_setting {
    BELUGA_ID,             ///< Node ID
    BELUGA_BOOTMODE,       ///< Boot mode
    BELUGA_POLL_RATE,      ///< Rate that UWB initiates ranging requests
    BELUGA_UWB_CHANNEL,    ///< UWB channel used
    BELUGA_BLE_TIMEOUT,    ///< Timeout before removing from neighbor list
    BELUGA_TX_POWER,       ///< UWB transmission power
    BELUGA_STREAMMODE,     ///< Neighbor list streaming option
    BELUGA_TWR,            ///< UWB ranging mode
    BELUGA_LEDMODE,        ///< LED mode
    BELUGA_OUT_FORMAT,     ///< Output format
    BELUGA_RANGE_EXTEND,   ///< Power amplifier state
    BELUGA_UWB_PHR,        ///< UWB PHR mode
    BELUGA_UWB_DATA_RATE,  ///< UWB data rate
    BELUGA_UWB_PULSE_RATE, ///< UWB pulse repetition rate
    BELUGA_UWB_PREAMBLE,   ///< UWB preamble length
    BELUGA_UWB_PAC,        ///< UWB PAC size
    BELUGA_UWB_NSSFD,      ///< UWB SFD length option
    BELUGA_PAN_ID,         ///< UWB PAN ID
    BELUGA_RESERVED        ///< Reserved settings enumerator indicating the last
                           ///< setting
};

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
