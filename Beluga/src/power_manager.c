//
// Created by tom on 9/6/24.
//

#include <zephyr/kernel.h>
#include <power_manager.h>
#include <init_main.h>
#include <resp_main.h>
#include <list_neighbors.h>
#include <app_leds.h>
#include <ble_app.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_EXISTS(DT_NODELABEL(lis2dh12)) && defined(CONFIG_BELUGA_USE_ACCEL)
#define USE_ACCEL
// Some other stuff
#elif DT_NODE_EXISTS(ZEPHYR_USER_NODE) && DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, wake_source_gpios)
#define WAKESOURCE GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, wake_source_gpios)
#elif DT_NODE_EXISTS(DT_ALIAS(wake_source))
#define WAKESOURCE GPIO_DT_SPEC_GET(DT_ALIAS(wake_source), gpios)
#else
#error "Wakeup source is not defined. Unable to wake from deep sleep."
#endif

#if defined(USE_ACCEL) || defined(WAKESOURCE)
#if defined(WAKESOURCE)
static const struct gpio_dt_spec wake_source = WAKESOURCE;
#endif

static void configure_wake_source(void) {
#if defined(USE_ACCEL)
    // TODO: Use accelerometer
#else
    int rc = gpio_pin_configure_dt(&wake_source, GPIO_INPUT);
    if (rc < 0) {
        printk("Could not configure wake source (%d)\n", rc);
        return;
    }

    rc = gpio_pin_interrupt_configure_dt(&wake_source, GPIO_INT_LEVEL_ACTIVE);
    if (rc < 0) {
        printk("Could not configure wake source GPIO interrupt (%d)\n", rc);
    }
#endif
}
#else
#define configure_wake_source() do {} while (0)
#endif

static void sleep_dw1000(void) {
    if (get_uwb_led_state() == LED_UWB_ON) {
        // Stop UWB tasks
        k_sem_take(&k_sus_resp, K_FOREVER);
        k_sem_take(&k_sus_init, K_FOREVER);
    }

    // TODO: Place UWB into deep sleep mode
}

static void stop_ble(void) {
    if (get_ble_led_state() == LED_BLE_ON) {
        k_sem_take(&print_list_sem, K_FOREVER);
    }
    deinit_bt_stack();
}

void enter_deep_sleep(void) {
    sleep_dw1000();
    stop_ble();
    configure_wake_source();
    sys_poweroff();
}