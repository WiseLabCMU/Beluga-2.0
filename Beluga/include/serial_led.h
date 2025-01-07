/**
 * @file serial_led.c
 * @brief Serial LED Control for Communication Status Indication
 *
 * Implements the logic for controlling serial LEDs that indicate the
 * state of serial communication. It provides functions for configuring,
 * initializing, and updating the LED states based on transmission (TX) and
 * reception (RX) events.
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 * @date 12/10/24
 */

#ifndef BELUGA_SERIAL_LED_H
#define BELUGA_SERIAL_LED_H

#include <stdbool.h>

/**
 * Defines the possible states for the serial LEDs, representing
 * the transmission and reception states of the serial communication.
 * The state of the LEDs can be updated based on whether the system is
 * transmitting (TX) or receiving (RX) data.
 */
enum serial_led_state {
    LED_START_TX, ///< Serial transmission started
    LED_STOP_TX,  ///< Serial transmission ended
    LED_START_RX, ///< Serial reception started
    LED_STOP_RX   ///< Serial reception ended
};

/**
 * @brief Initializes the serial LEDs
 * @return 0 upon success
 * @return negative error code otherwise
 */
int serial_leds_init(void);

/**
 * @brief Updates the LED states based on the current serial communication state
 *
 * @param[in] state The serial communication state
 *
 * @return 0 upon success
 * @return -EINVAL for an invalid state
 * @return negative error code otherwise
 */
int serial_leds_update_state(enum serial_led_state state);

/**
 * @brief Enables or disables the serial LEDs
 * @param[in] enable `true` if the serial LEDs are to be enabled, `false` if the
 * serial LEDs are to be disabled.
 * @return 0 upon success
 * @return -ETIMEDOUT if unable to disable LED state updates within 10ms
 */
int set_serial_led_master_state(bool enable);

#endif // BELUGA_SERIAL_LED_H
