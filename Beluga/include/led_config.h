/**
 * @file led_config.h
 * @brief LED configuration for Beluga.
 *
 * This header file defines macros and functions for controlling LEDs within
 * Beluga. It includes conditional compilation for two LED configurations,
 * depending on the enabled project-specific configurations. The configurations
 * allow the LEDs to be either controlled by the application itself to indicate
 * the power states of the MCU, BLE, UWB, and power amplifiers, or they can be
 * controlled by the BLE app to indicate certain BLE states.
 *
 * The `CONFIG_BELUGA_LEDS` and `CONFIG_BELUGA_BLE_LEDS` configuration options
 * allow for selecting between application control of the LEDs and Bluetooth Low
 * Energy (BLE) control of the LEDs.
 *
 * The macros in this file handle LED state changes (turning LEDs on and off)
 * and initialization, including logging error messages if any operation fails.
 *
 * @data 6/18/2024
 *
 * @author Tom Schmitz
 */

#ifndef BELUGA_LED_CONFIG_H
#define BELUGA_LED_CONFIG_H

#if defined(CONFIG_DK_LIBRARY) &&                                              \
    (defined(CONFIG_BELUGA_LEDS) || defined(CONFIG_BELUGA_BLE_LEDS))

/**
 * Indicates that LEDs are enabled
 */
#define LED_SUPPORT_ENABLED

#include <dk_buttons_and_leds.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#else

/**
 * Placeholder macros when the LEDs are disabled
 */
#define DK_LED1 1
#define DK_LED2 2
#define DK_LED3 3
#define DK_LED4 4
#endif // defined(CONFIG_DK_LIBRARY) && (defined(CONFIG_BELUGA_LEDS) ||
       // defined(CONFIG_BELUGA_BLE_LEDS))

/**
 * LED used to indicate advertising state in BLE
 */
#define PERIPHERAL_ADVERTISING_LED DK_LED1

/**
 * LED used to indicate the peripheral connected state in BLE
 */
#define PERIPHERAL_CONNECTED_LED DK_LED2

/**
 * LED used to indicate the scanning state in BLE
 */
#define CENTRAL_SCANNING_LED DK_LED3

/**
 * LED used to indicate the central connected state in BLE
 */
#define CENTRAL_CONNECTED_LED DK_LED4

/**
 * LED used to indicate the UWB power state
 */
#define UWB_LED DK_LED3

/**
 * LED used to indicate the BLE power state
 */
#define BLE_LED DK_LED4

/**
 * LED used to indicate the power amplifier state
 */
#define PWRAMP_LED DK_LED1

/**
 * LED used to indicate the MCU power state
 */
#define POWER_LED DK_LED2

/**
 * Used for early exit with no return value when controlling LEDs.
 */
#define LED_EARLY_EXIT_VOID 0

/**
 * Used for early exit with an integer return value when controlling LEDs.
 */
#define LED_EARLY_EXIT_INT 1

#ifdef LED_SUPPORT_ENABLED

/**
 * @brief Helper macro for controlling an LED without returning a value.
 *
 * This macro calls a function (`_func`) to control the LED state and logs an
 * error if the function returns a non-zero value (indicating failure).
 *
 * @param[in] _func Function to call for controlling the LED (e.g.,
 * `dk_set_led_on`).
 * @param[in] LED The LED to control.
 */
#define _LED_ACTION_NO_RET(_func, LED)                                         \
    do {                                                                       \
        int err = _func(LED);                                                  \
        if (err) {                                                             \
            LOG_ERR("Unable to update LED using " #_func "(). Returned (%d)",  \
                    err);                                                      \
        }                                                                      \
    } while (0)

/**
 * @brief Helper macro for controlling an LED with early return if there is an
 * error.
 *
 * This macro calls a function (`_func`) to control the LED state and returns
 * early if an error occurs.
 *
 * @param[in] _func Function to call for controlling the LED (e.g.,
 * `dk_set_led_on`).
 * @param[in] LED The LED to control.
 */
#define _LED_ACTION_RET_VOID(_func, LED)                                       \
    do {                                                                       \
        int err = _func(LED);                                                  \
        if (err) {                                                             \
            LOG_ERR("Unable to update LED using " #_func "(). Returned (%d)",  \
                    err);                                                      \
            return;                                                            \
        }                                                                      \
    } while (0)

/**
 * @brief Helper macro for controlling an LED and returning the error code on
 * failure.
 *
 * This macro calls a function (`_func`) to control the LED state and returns
 * the error code if the function fails.
 *
 * @param[in] _func Function to call for controlling the LED (e.g.,
 * `dk_set_led_on`).
 * @param[in] LED The LED to control.
 */
#define _LED_ACTION_RET_ERR(_func, LED)                                        \
    do {                                                                       \
        int err = _func(LED);                                                  \
        if (err) {                                                             \
            LOG_ERR("Unable to update LED using " #_func "(). Returned (%d)",  \
                    err);                                                      \
            return err;                                                        \
        }                                                                      \
    } while (0)

/**
 * @brief Macro for controlling an LED based on a conditional argument.
 *
 * This macro checks the argument `arg` and either calls `_LED_ACTION_RET_ERR`
 * or
 * `_LED_ACTION_RET_VOID` based on whether `arg` is set to `LED_EARLY_EXIT_INT`
 * (indicating an early exit with an error code) or `LED_EARLY_EXIT_VOID`
 * (indicating an early exit from a void function).
 *
 * @param[in] _func Function to call for controlling the LED (e.g.,
 * `dk_set_led_on`).
 * @param[in] LED The LED to control.
 * @param[in] arg Conditional argument that determines return behavior
 * (`LED_EARLY_EXIT_INT` or `LED_EARLY_EXIT_VOID`).
 */
#define _LED_ACTION_RET(_func, LED, arg)                                       \
    COND_CODE_1(arg, (_LED_ACTION_RET_ERR(_func, LED)),                        \
                (_LED_ACTION_RET_VOID(_func, LED)))

/**
 * @brief Macro to generate the correct code for controlling an LED (on or off).
 *
 * This macro generates the correct LED control code based on whether additional
 * arguments are passed to specify an early exit condition.
 *
 * @param LED The LED to control.
 * @param _func The function to call for controlling the LED (e.g.,
 * `dk_set_led_on`).
 * @param ... Additional arguments to determine behavior.
 */
#define _LED_GENERATE_CODE(LED, _func, ...)                                    \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (_LED_ACTION_NO_RET(_func, LED)),       \
                (_LED_ACTION_RET(_func, LED, GET_ARG_N(1, __VA_ARGS__))))

/**
 * @brief Macro to turn the specified LED on.
 *
 * This macro uses `dk_set_led_on` to turn on the specified LED.
 *
 * @param[in] LED The LED to turn on.
 * @param[in] ... Additional arguments passed to control behavior.
 */
#define LED_ON(LED, ...) _LED_GENERATE_CODE(LED, dk_set_led_on, __VA_ARGS__)

/**
 * @brief Macro to turn the specified LED off.
 *
 * This macro uses `dk_set_led_off` to turn off the specified LED.
 *
 * @param[in] LED The LED to turn off.
 * @param[in] ... Additional arguments passed to control behavior.
 */
#define LED_OFF(LED, ...) _LED_GENERATE_CODE(LED, dk_set_led_off, __VA_ARGS__)

/**
 * @brief Macro to initialize the LEDs.
 *
 * This macro initializes the LEDs using `dk_leds_init()` and turns off all LEDs
 * during initialization. If initialization fails, it logs an error and returns
 * 0.
 */
#define LED_INIT()                                                             \
    do {                                                                       \
        int _ledErr = dk_leds_init();                                          \
        if (_ledErr) {                                                         \
            LOG_ERR("LEDs init failed (err %d)\n", _ledErr);                   \
            return 0;                                                          \
        }                                                                      \
        LED_OFF(DK_LED1, LED_EARLY_EXIT_INT);                                  \
        LED_OFF(DK_LED2, LED_EARLY_EXIT_INT);                                  \
        LED_OFF(DK_LED2, LED_EARLY_EXIT_INT);                                  \
        LED_OFF(DK_LED3, LED_EARLY_EXIT_INT);                                  \
        LED_OFF(DK_LED4, LED_EARLY_EXIT_INT);                                  \
        LOG_INF("LEDs initialized");                                           \
    } while (0)
#else
/**
 * Placeholder macro when LEDs are not enabled
 */
#define LED_ON(LED, ...)  (void)0

/**
 * Placeholder macro when LEDs are not enabled
 */
#define LED_OFF(LED, ...) LED_ON(LED)

/**
 * Placeholder macro when LEDs are not enabled
 */
#define LED_INIT()        LOG_INF("LEDs disabled");
#endif // LED_SUPPORT_ENABLED

#if defined(CONFIG_BELUGA_LEDS)

/**
 * Macros for controlling LEDs when the LEDs are used to indicate power states
 *
 * @{
 */
#define APP_LED_ON(LED, ...)  LED_ON(LED, __VA_ARGS__)
#define APP_LED_OFF(LED, ...) LED_OFF(LED, __VA_ARGS__)
#define BLE_LED_ON(LED)       (void)0
#define BLE_LED_OFF(LED)      BLE_LED_ON(LED)
/**
 * @}
 */

#else

/**
 * Macros for controlling LEDs when the LEDs are used to indicate BLE states
 *
 * @{
 */
#define APP_LED_ON(LED)       (void)0
#define APP_LED_OFF(LED)      APP_LED_ON(LED)
#define BLE_LED_ON(LED, ...)  LED_ON(LED, __VA_ARGS__)
#define BLE_LED_OFF(LED, ...) LED_OFF(LED, __VA_ARGS__)
/**
 * @}
 */

#endif // defined(CONFIG_BELUGA_LEDS)

#endif // BELUGA_LED_CONFIG_H
