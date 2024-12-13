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

#ifndef BELUGA_APP_LEDS_H
#define BELUGA_APP_LEDS_H

#include <stdbool.h>

/**
 * @enum led_state
 *
 * @brief Enum representing the possible states for various LEDs in the system.
 *
 * This enum defines the on/off states for the LEDs corresponding to different
 * components such as BLE, UWB, Power, and the power amplifier. It is used to
 * track and update the state of the LEDs.
 */
enum led_state {
    LED_BLE_ON,    ///< BLE LED is turned on
    LED_BLE_OFF,   ///< BLE LED is turned off
    LED_UWB_ON,    ///< UWB LED is turned on
    LED_UWB_OFF,   ///< UWB LED is turned off
    LED_POWER_ON,  ///< Power LED is turned on
    LED_POWER_OFF, ///< Power LED is turned off
    LED_PWRAMP_ON, ///< Power amplifier LED is turned on
    LED_PWRAMP_OFF ///< Power amplifier LED is turned off
};

/**
 * @brief Updates the state of the specified LED based on the given LED state.
 *
 * @param[in] update The new state to apply to the LED (e.g., LED_BLE_ON,
 * LED_UWB_OFF).
 */
void update_led_state(enum led_state update);

/**
 * @brief Turns off all LEDs and disables LED control.
 */
void all_leds_off(void);

/**
 * @brief Restores previously set LED states and re-enables LED control.
 */
void restore_led_states(void);

/**
 * @brief Returns the current state of the BLE LED.
 *
 * @return The current state of the BLE LED (LED_BLE_ON or LED_BLE_OFF).
 *
 * @note If the LED master state is off, this will still return the current
 * state of the BLE LED as if the LED master state is on
 */
enum led_state get_ble_led_state(void);

/**
 * @brief Returns the current state of the UWB LED.
 *
 * @return The current state of the UWB LED (LED_UWB_ON or LED_UWB_OFF).
 *
 * @note If the LED master state is off, this will still return the current
 * state of the UWB LED as if the LED master state is on
 */
enum led_state get_uwb_led_state(void);

/**
 * @brief Returns the current state of the Power LED.
 *
 * @return The current state of the Power LED (LED_POWER_ON or LED_POWER_OFF).
 *
 * @note If the LED master state is off, this will still return the current
 * state of the Power LED as if the LED master state is on
 */
enum led_state get_power_led_state(void);

/**
 * @brief Returns the current state of the unused PWRAMP LED.
 *
 * @return The current state of the PWRAMP LED (LED_PWRAMP_ON or
 * LED_PWRAMP_OFF).
 *
 * @note If the LED master state is off, this will still return the current
 * state of the Amplifier LED as if the LED master state is on
 */
enum led_state get_pwramp_led_state(void);

/**
 * @brief Checks if any LEDs are currently on.
 *
 * @return true if LED master state is on
 * @return false if the LED master state is off
 */
bool are_leds_on(void);

#endif // BELUGA_APP_LEDS_H
