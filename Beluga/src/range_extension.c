/**
 * @file range_extension.c
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

#include <app_leds.h>
#include <range_extension.h>
#include <stdio.h>
#include <utils.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/**
 * Logger for the range extension module
 */
LOG_MODULE_REGISTER(range_ext_logger, CONFIG_RANGE_EXTENSION_LOG_LEVEL);

/**
 * SKY FEM node that contains the GPIO pins
 */
#define SKY_GPIOS DT_NODELABEL(sky_fem_gpios)

#if defined(CONFIG_BELUGA_RANGE_EXTENSION) && DT_NODE_EXISTS(SKY_GPIOS)
#include <ble_app.h>
#include <deca_device_api.h>
#include <zephyr/drivers/gpio.h>

/**
 * Indicates that the antenna selct pin is present
 */
#define ANTENNA_SELECT DT_NODE_HAS_PROP(SKY_GPIOS, ant_sel_gpios)

#if !DT_NODE_HAS_PROP(SKY_GPIOS, csd_gpios)
#error "Shutdown gpio property (csd-gpios) is missing"
#endif

/**
 * Indicates that the pin that allows the FEM amplifiers to be bypassed is
 * present
 */
#define RF_BYPASS DT_NODE_HAS_PROP(SKY_GPIOS, cps_gpios)

/**
 * Indicates that the pin that controls the gain is present
 */
#define HIGHLOW_POWER DT_NODE_HAS_PROP(SKY_GPIOS, chl_gpios)

/**
 * Helper macro for generating a struct member
 */
#define _GEN_STRUCT_MEMBER(member) const struct gpio_dt_spec member

/**
 * Generates a struct member given that the pin is present in the device tree
 *
 * @param[in] enabled Determines if the struct member is generated or not
 * @param[in] member The name of the member to generate
 */
#define GEN_STRUCT_MEMBER(enabled, member)                                     \
    COND_CODE_1(IS_ENABLED(enabled), (_GEN_STRUCT_MEMBER(member)), (EMPTY))

/**
 * Helper macro to generate the initializer for the specified member
 *
 * @param[in] member The member to generate the initializer for
 * @param[in] nodelabel The node from the device tree to get the GPIO pin from
 * @param[in] prop The property of the node that defines the GPIO pin used
 */
#define _INIT_STRUCT_MEMBER(member, nodelabel, prop)                           \
    .member = GPIO_DT_SPEC_GET(nodelabel, prop)

/**
 * Generates the initialization of a struct member given that the pin is present
 * in the device tree
 *
 * @param[in] enabled Determines if the struct member initialization is
 * generated or not
 * @param[in] member The member to generate the initializer for
 * @param[in] nodelabel The node from the device tree to get the GPIO pin from
 * @param[in] prop The property of the node that defines the GPIO pin used
 */
#define INIT_STRUCT_MEMBER(enabled, member, nodelabel, prop)                   \
    COND_CODE_1(IS_ENABLED(enabled),                                           \
                (_INIT_STRUCT_MEMBER(member, nodelabel, prop)), (EMPTY))

/**
 * @brief Struct to hold GPIO specifications for various FEM pins.
 *
 * This structure contains the GPIO specifications for the pins controlling
 * the antenna select, shutdown, bypass, and power level output functions of the
 * FEM.
 *
 * @details
 * - `ant_sel`: GPIO for the antenna select pin.
 * - `shutdown`: GPIO for the shutdown pin.
 * - `bypass`: GPIO for the amplifier bypass pin
 * - `power`: GPIO for the high/low power control pin.
 */
static struct fem_gpios {
    GEN_STRUCT_MEMBER(ANTENNA_SELECT, ant_sel);
    GEN_STRUCT_MEMBER(1, shutdown);
    GEN_STRUCT_MEMBER(RF_BYPASS, bypass);
    GEN_STRUCT_MEMBER(HIGHLOW_POWER, power);
} _fem_gpios = {LIST_DROP_EMPTY(
    INIT_STRUCT_MEMBER(ANTENNA_SELECT, ant_sel, SKY_GPIOS, ant_sel_gpios),
    INIT_STRUCT_MEMBER(1, shutdown, SKY_GPIOS, csd_gpios),
    INIT_STRUCT_MEMBER(RF_BYPASS, bypass, SKY_GPIOS, cps_gpios),
    INIT_STRUCT_MEMBER(HIGHLOW_POWER, power, SKY_GPIOS, chl_gpios), )};

/**
 * @brief Helper macro to initialize a specific pin in the FEM GPIO container.
 *
 * This macro checks if the GPIO port for a specific pin is ready, and if so, it
 * configures the pin with the given configuration. If the pin is not ready or
 * the configuration fails, an error is logged and the calling function returns
 * early with an error.
 *
 * @param[in] container The GPIO container struct that holds the pin
 * information.
 * @param[in] attr The specific attribute (pin) to configure within the
 * container.
 * @param[in] config The configuration to apply to the pin.
 */
#define _INIT_FEM_PIN(container, attr, config)                                 \
    do {                                                                       \
        int err;                                                               \
        if (!device_is_ready((container).attr.port)) {                         \
            LOG_ERR(#attr " was not ready\n");                                 \
            return -ENODEV;                                                    \
        }                                                                      \
        err = gpio_pin_configure_dt(&(container).attr, config);                \
        if (err) {                                                             \
            LOG_ERR(#attr " configure (%d)\n", err);                           \
            return err;                                                        \
        }                                                                      \
    } while (0)

/**
 * Generates the initialization code for a pin in the FEM GPIO container given
 * that the pin is present in the device tree.
 *
 * @param[in] enabled Determines if the pin initialization code is generated or
 * not.
 * @param[in] container The GPIO container struct that holds the pin
 * information.
 * @param[in] attr The specific attribute (pin) to configure within the
 * container.
 * @param[in] config The configuration to apply to the pin.
 */
#define INIT_FEM_PIN(enabled, container, attr, config)                         \
    COND_CODE_1(IS_ENABLED(enabled), (_INIT_FEM_PIN(container, attr, config)), \
                (EMPTY))

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
int init_range_extension(void) {
    INIT_FEM_PIN(ANTENNA_SELECT, _fem_gpios, ant_sel, GPIO_OUTPUT_INACTIVE);
    INIT_FEM_PIN(1, _fem_gpios, shutdown, GPIO_OUTPUT_LOW);
    INIT_FEM_PIN(RF_BYPASS, _fem_gpios, bypass, GPIO_OUTPUT_HIGH);
    INIT_FEM_PIN(HIGHLOW_POWER, _fem_gpios, power, GPIO_OUTPUT_LOW);
    return 0;
}

/**
 * @brief Updates the specified FEM pin with a new value.
 *
 * This macro updates a specific pin (given by the `container` and `attr`) to a
 * new value, and handles errors by logging a message if the update fails.
 *
 * @param[in] container The GPIO container that holds the pin specification.
 * @param[in] attr The specific attribute (pin) within the container to be
 * updated.
 * @param[in] value The value to set for the pin.
 * @param[out] ret The return code that will be set to the error code in case of
 * failure.
 */
#define _UPDATE_FEM_PIN(container, attr, value, ret)                           \
    do {                                                                       \
        int err = gpio_pin_set_dt(&(container).attr, (value));                 \
        if (err != 0) {                                                        \
            LOG_ERR("Failed to update " #attr " to " #value " (err %d)", err); \
            (ret) = err;                                                       \
        }                                                                      \
    } while (0)

/**
 * @brief Handles the case when a FEM pin is not specified in the device tree
 *
 * This macro Sets the return code (`ret`) to `-ENODEV`, indicating that the
 * pin is not present
 *
 * @param[out] ret The return code that will be set to `-ENODEV` to indicate
 * the feature is not supported.
 */
#define FEM_PIN_NOTSUP(ret)                                                    \
    do {                                                                       \
        (ret) = -ENODEV;                                                       \
    } while (0)

/**
 * Generates the update code for a pin in the FEM GPIO container given that the
 * pin is present in the device tree.
 *
 * @param[in] enabled Determines if the pin update code is generated or not.
 * @param[in] container The GPIO container that holds the pin specification.
 * @param[in] attr The specific attribute (pin) within the container to be
 * updated.
 * @param[in] value The value to set for the pin.
 * @param[out] ret The return code that will be set based on the result of the
 * operation.
 */
#define UPDATE_FEM_PIN(enabled, container, attr, value, ret)                   \
    COND_CODE_1(IS_ENABLED(enabled),                                           \
                (_UPDATE_FEM_PIN(container, attr, value, ret)),                \
                (FEM_PIN_NOTSUP(ret)))

#define _TOGGLE_UWB_PA(uwb_pa_)                                                \
    do {                                                                       \
        dwt_setlnapamode(0, uwb_pa_);                                          \
        if (uwb_pa_) {                                                         \
            dwt_setfinegraintxseq(false);                                      \
        } else {                                                               \
            dwt_setfinegraintxseq(true);                                       \
        }                                                                      \
    } while (0)

/**
 * Generates the code to toggle on and off the external UWB amplifier. If the
 * UWB external amp is disabled in the config, then an empty statement is
 * inserted.
 *
 * @param[in] uwb_pa_ The external power amp setting for the DW1000
 */
#define TOGGLE_UWB_AMP(uwb_pa_)                                                \
    IF_ENABLED(IS_ENABLED(CONFIG_UWB_ENABLE_PA), (_TOGGLE_UWB_PA(uwb_pa_)))

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
int update_power_mode(enum power_mode mode) {
    int ret = 0, uwb_pa = ((uint8_t)mode) & UINT8_C(1);
    bool ble_state = save_and_disable_bluetooth();

    switch (mode) {
    case POWER_MODE_EXTERNAL_AMPS_OFF:
    case POWER_MODE_BYPASS: {
        UPDATE_FEM_PIN(RF_BYPASS, _fem_gpios, bypass, 1, ret);
        break;
    }
    case POWER_MODE_LOW_NO_UWB:
    case POWER_MODE_LOW: {
        UPDATE_FEM_PIN(RF_BYPASS, _fem_gpios, bypass, 0, ret);
        UPDATE_FEM_PIN(HIGHLOW_POWER, _fem_gpios, power, 0, ret);
        break;
    }
    case POWER_MODE_HIGH_NO_UWB:
    case POWER_MODE_HIGH: {
        UPDATE_FEM_PIN(RF_BYPASS, _fem_gpios, bypass, 0, ret);
        UPDATE_FEM_PIN(HIGHLOW_POWER, _fem_gpios, power, 1, ret);
        break;
    }
    default:
        ret = -EINVAL;
        break;
    }

    restore_bluetooth(ble_state);

    if (ret == 0 || ret == -ENODEV) {
        TOGGLE_UWB_AMP(uwb_pa);
    }

    return ret;
}

/**
 * @brief Selects the active antenna
 *
 * @param[in] antenna The antenna to select
 *
 * @return 0 on success
 * @return -EINVAL if invalid antenna is selected
 * @return -ENOTSUP if antenna selection is not supported
 */
int select_antenna(int32_t antenna) {
#if ANTENNA_SELECT
    bool ble_state = save_and_disable_bluetooth();
    int err = 0;

    switch (antenna) {
    case 1:
        err = gpio_pin_set_dt(&_fem_gpios.ant_sel, 0);
        break;
    case 2:
        err = gpio_pin_set_dt(&_fem_gpios.ant_sel, 1);
        break;
    default:
        err = -EINVAL;
        break;
    }

    restore_bluetooth(ble_state);

    if (err == 0) {
        LOG_ERR("Could not change antenna (%d)", err);
    }
    return err;
#else
    ARG_UNUSED(antenna);
    return -ENOTSUP;
#endif
}

/**
 * @brief Controls if the FEM (Front-End Module) is powered down or not.
 *
 * @param[in] shutdown `true` to power down the FEM, `false` to power on the
 * FEM.
 *
 * @return 0 upon success
 * @return negative error code upon failure
 */
int update_fem_shutdown_state(bool shutdown) {
    int err = gpio_pin_set_dt(&_fem_gpios.shutdown, shutdown);

    if (err != 0) {
        LOG_ERR("Could not shut down FEM (%d)", err);
    }

    return err;
}

#else

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
int init_range_extension(void) {
    LOG_INF("Range extension disabled");
    return 0;
}

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
int update_power_mode(enum power_mode mode) {
    if (mode == POWER_MODE_BYPASS) {
        return 0;
    }
    return -ENOTSUP;
}

/**
 * @brief Selects the active antenna
 *
 * @param[in] antenna The antenna to select
 *
 * @return 0 on success
 * @return -EINVAL if invalid antenna is selected
 * @return -ENOTSUP if antenna selection is not supported
 */
int select_antenna(int32_t ant) {
    ARG_UNUSED(ant);
    return -ENOTSUP;
}

/**
 * @brief Controls if the FEM (Front-End Module) is powered down or not.
 *
 * @param[in] shutdown `true` to power down the FEM, `false` to power on the
 * FEM.
 *
 * @return 0 upon success
 * @return negative error code upon failure
 */
int update_fem_shutdown_state(bool shutdown) {
    ARG_UNUSED(shutdown);
    return 0;
}
#endif // defined(CONFIG_BELUGA_RANGE_EXTENSION) && DT_NODE_EXISTS(SKY_GPIOS)
