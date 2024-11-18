//
// Created by tom on 9/6/24.
//

#include <app_leds.h>
#include <ble_app.h>
#include <initiator.h>
#include <list_neighbors.h>
#include <power_manager.h>
#include <responder.h>
#include <spi.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/poweroff.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_EXISTS(DT_NODELABEL(lis2dh12_i2c)) &&                              \
    defined(CONFIG_BELUGA_USE_ACCEL)
#define USE_ACCEL
#define ACCEL_NODE DT_NODELABEL(lis2dh12_i2c)
#elif DT_NODE_EXISTS(ZEPHYR_USER_NODE) &&                                      \
    DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, wake_source_gpios)
#define WAKESOURCE GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, wake_source_gpios)
#elif DT_NODE_EXISTS(DT_ALIAS(wake_source))
#define WAKESOURCE GPIO_DT_SPEC_GET(DT_ALIAS(wake_source), gpios)
#else
#warning "Wakeup source is not defined. Unable to wake from deep sleep."
#endif

#if defined(USE_ACCEL)
void accel_trigger_handler(const struct device *dev,
                           const struct sensor_trigger *trig) {}
#elif defined(WAKESOURCE)
static const struct gpio_dt_spec wake_source = WAKESOURCE;
#endif

// #if defined(USE_ACCEL) || defined(WAKESOURCE)

static void configure_wake_source(void) {
    // #if defined(USE_ACCEL)
    //     const struct device *accel = DEVICE_DT_GET(ACCEL_NODE);
    //     struct sensor_trigger trig;
    //
    //     if (!device_is_ready(accel)) {
    //         printk("Accelerometer is not available\n");
    //         return;
    //     }
    //
    //     trig.type = SENSOR_TRIG_DELTA;
    //     trig.chan = SENSOR_CHAN_ACCEL_XYZ;
    //
    //     if (sensor_trigger_set(accel, &trig, accel_trigger_handler)) {
    //         printk("Could not set trigger\n");
    //     }
    // #else
    //     int rc = gpio_pin_configure_dt(&wake_source, GPIO_INPUT);
    //     if (rc < 0) {
    //         printk("Could not configure wake source (%d)\n", rc);
    //         return;
    //     }
    //
    //     rc = gpio_pin_interrupt_configure_dt(&wake_source,
    //     GPIO_INT_LEVEL_ACTIVE); if (rc < 0) {
    //         printk("Could not configure wake source GPIO interrupt (%d)\n",
    //         rc);
    //     }
    // #endif
}
// #else
// #define configure_wake_source()
//     do {
//     } while (0)
// #endif

static void sleep_dw1000(void) {
    if (get_uwb_led_state() == LED_UWB_ON) {
        // Stop UWB tasks
        k_sem_take(&k_sus_resp, K_FOREVER);
        k_sem_take(&k_sus_init, K_FOREVER);
    }

    dwt_configuresleep(0, DWT_SLP_EN | DWT_WAKE_CS);
    dwt_entersleep();
    shutdown_spi();
}

static void stop_ble(void) {
    if (get_ble_led_state() == LED_BLE_ON) {
        k_sem_take(&print_list_sem, K_FOREVER);
    }
    deinit_bt_stack();
}

void enter_deep_sleep(void) {
    all_leds_off();
    stop_ble();
    sleep_dw1000();
    configure_wake_source();
    sys_poweroff();
}
