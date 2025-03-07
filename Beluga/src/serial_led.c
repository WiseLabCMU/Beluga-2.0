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

#include <serial_led.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

/**
 * Logger for the serial LEDs
 */
LOG_MODULE_REGISTER(serial_led, CONFIG_SERIAL_LED_LOG_LEVEL);

/**
 * The node defining the LEDs being used for serial communication
 */
#define SERIAL_LEDS_NODE DT_PATH(serial_leds)

#if defined(CONFIG_SERIAL_LEDS) && DT_NODE_EXISTS(SERIAL_LEDS_NODE)
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/**
 * GPIO bank 0 device
 */
#define GPIO0_DEV DEVICE_DT_GET(DT_NODELABEL(gpio0))

/**
 * GPIO bank 1 device
 */
#define GPIO1_DEV DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio1))

/**
 * Helper macro for getting gpio devicetree specs and inserting commas in
 * between
 */
#define GPIO_SPEC_AND_COMMA(led) GPIO_DT_SPEC_GET(led, gpios),

/**
 * The GPIO specs for the serial LEDs
 */
static const struct gpio_dt_spec leds[] = {
#if DT_NODE_EXISTS(SERIAL_LEDS_NODE)
    DT_FOREACH_CHILD(SERIAL_LEDS_NODE, GPIO_SPEC_AND_COMMA)
#endif
};

/**
 * Bit mask for no LEDs
 */
#define LEDS_NONE_MASK 0x0

/**
 * Bit mask for all the LEDs
 */
#define LEDS_ALL_MASK (((uint32_t)1 << ARRAY_SIZE(leds)) - 1)

/**
 * The bit mask for the TX LED
 */
#define LED_TX_MASK BIT(0)

/**
 * The bit mask for the RX LED
 */
#define LED_RX_MASK (LEDS_ALL_MASK & BIT(1))

/**
 * Flag indicating whether to enable/disable the serial LEDs
 */
static atomic_t serial_led_on = ATOMIC_INIT(1);

/**
 * @brief Updates the states of the serial LEDs to on or off
 *
 * @param[in] leds_on_mask The LEDs to turn on
 * @param[in] leds_off_mask The LEDs to turn off
 *
 * @note If an LED is not specified in either mask, that LED's
 * state is not updated
 *
 * @return 0 upon success
 * @return -EINVAL if either mask is invalid
 * @return negative error code otherwise
 */
static int serial_leds_set_state(uint32_t leds_on_mask,
                                 uint32_t leds_off_mask) {
    if ((leds_on_mask & ~LEDS_ALL_MASK) != 0 ||
        (leds_off_mask & ~LEDS_ALL_MASK) != 0) {
        return -EINVAL;
    }

    for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
        int val, err;

        if (BIT(i) & leds_on_mask) {
            val = 1;
        } else if (BIT(i) & leds_off_mask) {
            val = 0;
        } else {
            continue;
        }

        err = gpio_pin_set_dt(&leds[i], val);
        if (err) {
            LOG_ERR("Cannot write LED gpio");
            return err;
        }
    }

    return 0;
}

/**
 * @brief Initializes the serial LEDs
 * @return 0 upon success
 * @return negative error code otherwise
 */
int serial_leds_init(void) {
    int err;

    for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
        err = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT);
        if (err) {
            LOG_ERR("Cannot configure LED gpio (%d)", err);
            return err;
        }
    }

    return serial_leds_set_state(LEDS_NONE_MASK, LEDS_ALL_MASK);
}

/**
 * Reference counter indicating how many threads are actively updating thread
 * states
 */
atomic_t ref_count = ATOMIC_INIT(0);

/**
 * @brief Updates the LED states based on the current serial communication state
 *
 * @param[in] state The serial communication state
 *
 * @return 0 upon success
 * @return -EINVAL for an invalid state
 * @return negative error code otherwise
 */
int serial_leds_update_state(enum serial_led_state state) {
    int err;

    if (!atomic_test_bit(&serial_led_on, 0)) {
        return 0;
    }

    atomic_inc(&ref_count);

    switch (state) {
    case LED_START_TX:
        err = serial_leds_set_state(LED_TX_MASK, LEDS_NONE_MASK);
        break;
    case LED_STOP_TX:
        err = serial_leds_set_state(LEDS_NONE_MASK, LED_TX_MASK);
        break;
    case LED_START_RX:
        err = serial_leds_set_state(LED_RX_MASK, LEDS_NONE_MASK);
        break;
    case LED_STOP_RX:
        err = serial_leds_set_state(LEDS_NONE_MASK, LED_RX_MASK);
        break;
    default:
        err = -EINVAL;
        break;
    }

    atomic_dec(&ref_count);

    return err;
}

/**
 * @brief Enables or disables the serial LEDs
 * @param[in] enable `true` if the serial LEDs are to be enabled, `false` if the
 * serial LEDs are to be disabled.
 * @return 0 upon success
 * @return -ETIMEDOUT if unable to disable LED state updates within 10ms
 */
int set_serial_led_master_state(bool enable) {
    int ret = 0;

    if (enable) {
        atomic_set_bit(&serial_led_on, 0);
    } else {
        int wait = 10;

        atomic_clear_bit(&serial_led_on, 0);

        while (atomic_get(&ref_count) != 0 && wait != 0) {
            k_sleep(K_MSEC(1));
            wait -= 1;
        }

        if (wait <= 0) {
            LOG_ERR("Ref count wait timed out");
            ret = -ETIMEDOUT;
        }

        serial_leds_set_state(LEDS_NONE_MASK, LEDS_ALL_MASK);
    }

    return ret;
}

#else

/**
 * @brief Initializes the serial LEDs
 * @return 0 upon success
 * @return negative error code otherwise
 */
int serial_leds_init(void) {
    LOG_WRN("Not implemented");
    return 0;
}

/**
 * @brief Updates the LED states based on the current serial communication state
 *
 * @param[in] state The serial communication state
 *
 * @return 0 upon success
 * @return -EINVAL for an invalid state
 * @return negative error code otherwise
 */
int serial_leds_update_state(enum serial_led_state state) {
    ARG_UNUSED(state);
    LOG_WRN("Not implemented");
    return 0;
}

/**
 * @brief Enables or disables the serial LEDs
 * @param[in] enable `true` if the serial LEDs are to be enabled, `false` if the
 * serial LEDs are to be disabled.
 * @return 0 upon success
 * @return -ETIMEDOUT if unable to disable LED state updates within 10ms
 */
int set_serial_led_master_state(bool enable) {
    ARG_UNUSED(enable);
    LOG_WRN("Not implemented");
    return 0;
}
#endif // defined(CONFIG_SERIAL_LEDS) &&  DT_NODE_EXISTS(SERIAL_LEDS_NODE)
