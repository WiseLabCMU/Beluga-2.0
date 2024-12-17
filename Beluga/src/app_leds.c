/**
 * @file app_leds.c
 * @version 1.0
 * @date 7/10/24
 *
 * @brief This file provides the implementation of functions to control and
 * manage LEDs in the application. It includes functions to update the state of
 * individual LEDs, turn off all LEDs, restore previous LED states, and query
 * the status of specific LEDs. The LEDs are controlled based on various
 * operational states such as BLE, UWB, Power, and Amplifier.
 *
 * @author Tom Schmitz
 */

#include <app_leds.h>
#include <deca_device_api.h>
#include <led_config.h>
#include <serial_led.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_instance.h>
#include <zephyr/sys/util.h>

/**
 * Logger for the application LEDs
 *
 * @note The LED macros have logger statements in them
 */
LOG_MODULE_REGISTER(app_leds, CONFIG_APP_LEDS_LOG_LEVEL);

/**
 * Bit representing the BLE LED in the `ledState` bitfield
 */
#define BLE_BIT 0

/**
 * Bit representing the UWB LED in the `ledState` bitfield
 */
#define UWB_BIT 1

/**
 * Bit representing the power amplifier LED in the `ledState` bitfield
 */
#define PWRAMP_BIT 2

/**
 * Bit representing the power status LED in the `ledState` bitfield
 */
#define POWER_BIT 3

/**
 * Definition for the master on state
 */
#define led_mode_on true

/**
 * Definition for the master off state
 */
#define led_mode_off false

/**
 * Bitfield for the individual LED states
 */
static uint8_t ledState = 0;

/**
 * The master state of the LEDs
 *
 * An individual LED state can be generalized in the following boolean
 * expression: `led_mode & individual_led_state`
 */
static bool led_mode = led_mode_on;

/**
 * @brief Turns on the specified LED if the current mode allows it.
 *
 * @param[in] led The identifier for the LED to be turned on (e.g., BLE, UWB,
 * POWER, etc.).
 */
static void led_on(uint32_t led) {
    if (led_mode) {
        APP_LED_ON(led);
    }
}

/**
 * @brief Updates the state of the specified LED based on the given LED state.
 *
 * @param[in] update The new state to apply to the LED (e.g., LED_BLE_ON,
 * LED_UWB_OFF).
 */
void update_led_state(enum led_state update) {
    switch (update) {
    case LED_BLE_ON:
        ledState |= BIT(BLE_BIT);
        led_on(BLE_LED);
        break;
    case LED_BLE_OFF:
        ledState &= ~(BIT(BLE_BIT));
        APP_LED_OFF(BLE_LED);
        break;
    case LED_UWB_ON:
        ledState |= BIT(UWB_BIT);
        led_on(UWB_LED);
        break;
    case LED_UWB_OFF:
        ledState &= ~(BIT(UWB_BIT));
        APP_LED_OFF(UWB_LED);
        break;
    case LED_POWER_ON:
        ledState |= BIT(POWER_BIT);
        led_on(POWER_LED);
        break;
    case LED_POWER_OFF:
        ledState &= ~(BIT(POWER_BIT));
        APP_LED_OFF(POWER_LED);
        break;
    case LED_PWRAMP_ON:
        ledState |= BIT(PWRAMP_BIT);
        led_on(PWRAMP_LED);
        break;
    case LED_PWRAMP_OFF:
        ledState &= ~(BIT(PWRAMP_BIT));
        APP_LED_OFF(PWRAMP_LED);
        break;
    default:
        LOG_ERR("Attempted to update invalid LED: %d", update);
    }
}

/**
 * @brief Turns off all LEDs and disables LED control.
 */
void all_leds_off(void) {
    led_mode = led_mode_off;
    APP_LED_OFF(BLE_LED);
    APP_LED_OFF(UWB_LED);
    APP_LED_OFF(PWRAMP_LED);
    APP_LED_OFF(POWER_LED);
    dwt_setleds(DWT_LEDS_DISABLE);
    set_serial_led_master_state(false);
}

/**
 * @brief Restores previously set LED states and re-enables LED control.
 */
void restore_led_states(void) {
    all_leds_off();

    if (ledState & BIT(BLE_BIT)) {
        APP_LED_ON(BLE_LED);
    }

    if (ledState & BIT(UWB_BIT)) {
        APP_LED_ON(UWB_LED);
    }

    if (ledState & BIT(PWRAMP_BIT)) {
        APP_LED_ON(PWRAMP_LED);
    }

    if (ledState & BIT(POWER_BIT)) {
        APP_LED_ON(POWER_LED);
    }
    led_mode = led_mode_on;
    dwt_setleds(DWT_LEDS_ENABLE);
    set_serial_led_master_state(true);
}

/**
 * @brief Returns the current state of the BLE LED.
 *
 * @return The current state of the BLE LED (LED_BLE_ON or LED_BLE_OFF).
 *
 * @note If the LED master state is off, this will still return the current
 * state of the BLE LED as if the LED master state is on
 */
enum led_state get_ble_led_state(void) {
    return (ledState & BIT(BLE_BIT)) ? LED_BLE_ON : LED_BLE_OFF;
}

/**
 * @brief Returns the current state of the UWB LED.
 *
 * @return The current state of the UWB LED (LED_UWB_ON or LED_UWB_OFF).
 *
 * @note If the LED master state is off, this will still return the current
 * state of the UWB LED as if the LED master state is on
 */
enum led_state get_uwb_led_state(void) {
    return (ledState & BIT(UWB_BIT)) ? LED_UWB_ON : LED_UWB_OFF;
}

/**
 * @brief Returns the current state of the Power LED.
 *
 * @return The current state of the Power LED (LED_POWER_ON or LED_POWER_OFF).
 *
 * @note If the LED master state is off, this will still return the current
 * state of the Power LED as if the LED master state is on
 */
enum led_state get_power_led_state(void) {
    return (ledState & BIT(POWER_BIT)) ? LED_POWER_ON : LED_POWER_OFF;
}

/**
 * @brief Returns the current state of the unused PWRAMP LED.
 *
 * @return The current state of the PWRAMP LED (LED_PWRAMP_ON or
 * LED_PWRAMP_OFF).
 *
 * @note If the LED master state is off, this will still return the current
 * state of the Amplifier LED as if the LED master state is on
 */
enum led_state get_pwramp_led_state(void) {
    return ledState & BIT(PWRAMP_BIT) ? LED_PWRAMP_ON : LED_PWRAMP_OFF;
}

/**
 * @brief Checks if any LEDs are currently on.
 *
 * @return true if LED master state is on
 * @return false if the LED master state is off
 */
bool are_leds_on(void) { return led_mode == led_mode_on; }
