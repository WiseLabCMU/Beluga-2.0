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
 * @brief Power modes for the BLE FEM
 */
enum ble_power_mode {
    POWER_MODE_BYPASS, ///< Bypass the PA and LNA
    POWER_MODE_LOW,    ///< Amplify the BLE by 10 dB
    POWER_MODE_HIGH    ///< Tell FCC to fuck off and amplify the BLE by 22 dB
};

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
int update_power_mode(enum ble_power_mode mode);

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
