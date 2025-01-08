/**
 * @file voltage_regulator.h
 * @brief Driver for controlling the external voltage regulator.
 *
 * This file provides the implementation for controlling and querying the
 * external voltage regulator supplying the microcontroller.
 *
 * The voltage regulator configuration and GPIO pins are obtained from the
 * devicetree. If the devicetree does not contain the necessary voltage
 * regulator node, the driver provides placeholder functions with warnings.
 *
 * @date 8/13/2024
 * @author Tom Schmitz
 */

#ifndef BELUGA_VOLTAGE_REGULATOR_H
#define BELUGA_VOLTAGE_REGULATOR_H

#include <stdbool.h>

/**
 * @brief Enum for defining the available voltage levels of the regulator.
 *
 * Defines the possible supply voltage levels that the voltage regulator can be
 * set to. These values are used to select the desired voltage level.
 */
enum voltage_level {
    VR_2V4, ///< 2.4V Supply Voltage
    VR_3V3, ///< 3.3V Supply Voltage
    VR_3V5  ///< 3.5V Supply Voltage
};

/**
 * @brief Initializes the voltage regulator
 * @return 0 upon success
 * @return -ENODEV if voltage regulator is not ready
 * @return negative error code otherwise
 */
int init_voltage_regulator(void);

/**
 * @brief Updates the supply voltage
 * @param[in] level The new voltage level
 * @return 0 upon success
 * @return -EINVAL for invalid level
 * @return -ENOTSUP if voltage regulator node is not present
 * @return negative error code otherwise
 */
int update_voltage_level(enum voltage_level level);

/**
 * @brief Retrieves the current supply voltage level
 * @return The current voltage level
 */
enum voltage_level get_current_voltage(void);

#endif // BELUGA_VOLTAGE_REGULATOR_H
