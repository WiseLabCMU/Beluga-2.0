/**
 * @file range_extension.h
 *
 * @brief Controls the Front-End Module (FEM) for BLE range extension and UWB
 * amplifier usage.
 *
 * This manages the configuration and operation of the Front-End Module (FEM)
 * used for BLE range extension, specifically for Skyworks FEMs. It controls the
 * power modes of the FEM, antenna selection, and communicates with the DW1000
 * to enable or disable its external amplifier. Based on the selected power
 * mode, the FEM's power state is adjusted, and the DW1000 is instructed to
 * start or stop using its external amplifier accordingly.
 *
 * Only Skyworks FEMs are supported.
 *
 * @date 8/5/24
 * @author Tom Schmitz
 */

#ifndef BELUGA_RANGE_EXTENSION_H
#define BELUGA_RANGE_EXTENSION_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Power modes for the BLE FEM and UWB amplifier
 */
enum power_mode {
    POWER_MODE_EXTERNAL_AMPS_OFF = 0, ///< All external amplifiers are turned
                                      ///< off. In other words, use this mode
                                      ///< when the FCC is inspecting...
    POWER_MODE_BYPASS = 1,     ///< Bypass the PA and LNA, but turn on the UWB
    POWER_MODE_LOW_NO_UWB = 2, ///< Amplify the BLE by 10 dB and do not
                               ///< amplify UWB
    POWER_MODE_LOW = 3,        ///< Amplify the BLE by 10 dB and turn on the UWB
                               ///< amplifier
    POWER_MODE_HIGH_NO_UWB = 4, ///< Tell FCC to fuck off and amplify the BLE by
                                ///< 22 dB and do not amplify the UWB
    POWER_MODE_HIGH = 5, ///< Tell FCC to fuck off and amplify everything by
                         ///< 20+ dB
};

/**
 * Macro to check if the UWB amplifier is turned on based on the power mode.
 *
 * @param[in] power_mode The current power mode that we are in
 */
#define IS_UWB_AMP_ON(power_mode)                                              \
    ((power_mode) == POWER_MODE_BYPASS || (power_mode) == POWER_MODE_LOW ||    \
     (power_mode) == POWER_MODE_HIGH)

/**
 * Macro that checks if the given channel can be amplified by the UWB amplifier
 *
 * @param[in] channel The channel to check
 */
#define UWB_AMP_CHANNEL(channel) IN_RANGE(channel, 2, 4)

/**
 * @brief Initializes the BLE FEM (Front-End Module) pins that are not
 * associated with TX and RX.
 *
 * This function initializes the necessary GPIO pins related to antenna
 * selection, shutdown, RF bypass, and power control for the range extension
 * feature of the BLE FEM.
 *
 * @return 0 on success.
 * @return -ENODEV if the device is not ready.
 * @return A negative error code upon failure to configure any pin.
 */
int init_range_extension(void);

/**
 * @brief Updates the power mode of the FEM and controls the external
 * amplifiers.
 *
 * This function adjusts the power settings of the front-end module (FEM) and
 * the external power amplifier (UWB PA) based on the specified power mode. It
 * enables or disables the external amplifiers as needed and switches the FEM
 * between bypass, low, or high power modes.
 *
 * The following power modes are supported:
 *  - `POWER_MODE_BYPASS`: Disables the FEM and external amplifier (UWB PA) and
 * bypasses the FEM.
 *  - `POWER_MODE_LOW`: Sets the FEM to low power mode and disables the external
 * amplifier (UWB PA).
 *  - `POWER_MODE_HIGH`: Sets the FEM to high power mode and enables the
 * external amplifier (UWB PA).
 *
 * @param[in] mode The desired power mode to be set.
 *
 * @return 0 on success
 * @return -EINVAL if the power mode is not recognized
 * @return negative error code otherwise
 */
int update_power_mode(enum power_mode mode);

/**
 * @brief Selects the active antenna
 *
 * @param[in] antenna The antenna to select
 *
 * @return 0 on success
 * @return -EINVAL if invalid antenna is selected
 * @return -ENOTSUP if antenna selection is not supported
 */
int select_antenna(int32_t ant);

/**
 * @brief Controls if the FEM (Front-End Module) is powered down or not.
 *
 * @param[in] shutdown `true` to power down the FEM, `false` to power on the
 * FEM.
 *
 * @return 0 upon success
 * @return negative error code upon failure
 */
int update_fem_shutdown_state(bool shutdown);

#endif // BELUGA_RANGE_EXTENSION_H
