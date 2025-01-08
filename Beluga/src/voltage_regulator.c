/**
 * @file voltage_regulator.c
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

#include <voltage_regulator.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

/**
 * Logger for the voltage regulator
 */
LOG_MODULE_REGISTER(vr_logger, CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL);

#if DT_NODE_EXISTS(DT_NODELABEL(vr_adjust))

/**
 * The current level of the voltage regulator
 */
static enum voltage_level currentLevel = VR_3V3;

/**
 * The voltage regulator device from the devicetree
 */
static const struct gpio_dt_spec _vr_adjust =
    GPIO_DT_SPEC_GET(DT_NODELABEL(vr_adjust), gpios);

/**
 * @brief Initializes the voltage regulator
 * @return 0 upon success
 * @return -ENODEV if voltage regulator is not ready
 * @return negative error code otherwise
 */
int init_voltage_regulator(void) {
    int err;
    if (!device_is_ready(_vr_adjust.port)) {
        LOG_ERR("Voltage regulator gpio not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&_vr_adjust, GPIO_DISCONNECTED);
    currentLevel = VR_3V3;
    if (err == 0) {
        LOG_INF("Voltage regulator configured");
    } else {
        LOG_ERR("An error occurred when configuring voltage regulator");
    }
    return err;
}

/**
 * @brief Updates the supply voltage
 * @param[in] level The new voltage level
 * @return 0 upon success
 * @return -EINVAL for invalid level
 * @return -ENOTSUP if voltage regulator node is not present
 * @return negative error code otherwise
 */
int update_voltage_level(enum voltage_level level) {
    int err;

    switch (level) {
    case VR_2V4:
        err = gpio_pin_configure_dt(&_vr_adjust, GPIO_OUTPUT_ACTIVE);
        break;
    case VR_3V3:
        err = gpio_pin_configure_dt(&_vr_adjust, GPIO_DISCONNECTED);
        break;
    case VR_3V5:
        err = gpio_pin_configure_dt(&_vr_adjust, GPIO_OUTPUT_INACTIVE);
        break;
    default:
        LOG_WRN("Received an invalid voltage level");
        err = -EINVAL;
        break;
    }

    if (err == 0) {
        currentLevel = level;
    }

    return err;
}

/**
 * @brief Retrieves the current supply voltage level
 * @return The current voltage level
 */
enum voltage_level get_current_voltage(void) { return currentLevel; }

#else

/**
 * @brief Initializes the voltage regulator
 * @return 0 upon success
 * @return -ENODEV if voltage regulator is not ready
 * @return negative error code otherwise
 */
int init_voltage_regulator(void) {
    LOG_WRN("No voltage regulator node present");
    return 0;
}

/**
 * @brief Updates the supply voltage
 * @param[in] level The new voltage level
 * @return 0 upon success
 * @return -EINVAL for invalid level
 * @return -ENOTSUP if voltage regulator node is not present
 * @return negative error code otherwise
 */
int update_voltage_level(enum voltage_level level) {
    ARG_UNUSED(level);
    return -ENOTSUP;
}

/**
 * @brief Retrieves the current supply voltage level
 * @return The current voltage level
 */
enum voltage_level get_current_voltage(void) {
    LOG_WRN("No voltage regulator node present. Returning default level");
    return VR_3V3;
}

#endif // DT_NODE_EXISTS(DT_NODELABEL(vr_adjust))
