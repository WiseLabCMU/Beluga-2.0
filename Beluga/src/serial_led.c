//
// Created by tom on 12/10/24.
//

#include <serial_led.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(serial_led, CONFIG_SERIAL_LED_LOG_LEVEL);

#define SERIAL_LEDS_NODE         DT_PATH(serial_leds)

#define GPIO0_DEV                DEVICE_DT_GET(DT_NODELABEL(gpio0))
#define GPIO1_DEV                DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio1))

#define GPIO_SPEC_AND_COMMA(led) GPIO_DT_SPEC_GET(led, gpios),

static const struct gpio_dt_spec leds[] = {
#if DT_NODE_EXISTS(SERIAL_LEDS_NODE)
    DT_FOREACH_CHILD(SERIAL_LEDS_NODE, GPIO_SPEC_AND_COMMA)
#endif
};

#define LEDS_NONE_MASK 0x0
#define LEDS_ALL_MASK  (((uint32_t)1 << ARRAY_SIZE(leds)) - 1)

#define LED_TX_MASK    BIT(0)
#define LED_RX_MASK    (LEDS_ALL_MASK & BIT(1))

static atomic_t serial_led_on = ATOMIC_INIT(1);

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

atomic_t ref_count = ATOMIC_INIT(0);

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
