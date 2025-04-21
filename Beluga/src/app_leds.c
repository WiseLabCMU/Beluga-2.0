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
static inline void led_on(uint32_t led) {
    if (led_mode) {
        APP_LED_ON(led);
    }
}

/**
 * Sets the appropriate LED's state
 * @param[in] bit The bit that represents the LED's state
 * @param[in] led The LED being updated
 * @param[in] state The new state of the LED
 */
static inline void update_led(uint8_t bit, uint32_t led, enum led_state state) {
    switch (state) {
    case LED_ON: {
        ledState |= bit;
        led_on(led);
        break;
    }
    case LED_OFF: {
        ledState &= ~bit;
        APP_LED_OFF(led);
    }
    default:
        __ASSERT_UNREACHABLE;
        break;
    }
}

/**
 * @brief Updates the state of the specified LED based on the given LED state.
 *
 * @param[in] led The led whose state is being updated
 * @param[in] update The new state to apply to the LED.
 *
 * @note If the master LED state is off, then this will not update the hardware
 * state of the LED; However, it will save the software state. The software
 * state is only reflected by the hardware when the master LED state is on.
 */
void update_led_state(enum led led, enum led_state update) {
    switch (led) {
    case LED_BLE: {
        update_led(BIT(BLE_BIT), BLE_LED, update);
        break;
    }
    case LED_UWB: {
        update_led(BIT(UWB_BIT), UWB_LED, update);
        break;
    }
    case LED_POWER: {
        update_led(BIT(POWER_BIT), POWER_LED, update);
        break;
    }
    case LED_PWRAMP: {
        update_led(BIT(PWRAMP_BIT), PWRAMP_LED, update);
        break;
    }
    default:
        __ASSERT_UNREACHABLE;
        break;
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
    return (ledState & BIT(BLE_BIT)) ? LED_ON : LED_OFF;
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
    return (ledState & BIT(UWB_BIT)) ? LED_ON : LED_OFF;
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
    return (ledState & BIT(POWER_BIT)) ? LED_ON : LED_OFF;
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
    return ledState & BIT(PWRAMP_BIT) ? LED_ON : LED_OFF;
}

/**
 * @brief Checks if any LEDs are currently on.
 *
 * @return true if LED master state is on
 * @return false if the LED master state is off
 */
bool are_leds_on(void) { return led_mode == led_mode_on; }
