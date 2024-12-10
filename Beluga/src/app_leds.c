//
// Created by tom on 7/10/24.
//

#include <app_leds.h>
#include <deca_device_api.h>
#include <led_config.h>
#include <serial_led.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_instance.h>
#include <zephyr/sys/util.h>

// The LED macros have logger statements in them
LOG_MODULE_REGISTER(app_leds, CONFIG_APP_LEDS_LOG_LEVEL);

#define BLE_BIT      0
#define UWB_BIT      1
#define PWRAMP_BIT   2
#define POWER_BIT    3

#define led_mode_on  true
#define led_mode_off false

static uint8_t ledState = 0;
static bool led_mode = led_mode_on;

static void led_on(uint32_t led) {
    if (led_mode) {
        APP_LED_ON(led);
    }
}

void update_led_state(enum led_state update) {
    switch (update) {
    case LED_BLE_ON:
        ledState |= UINT8_C(1) << BLE_BIT;
        led_on(BLE_LED);
        break;
    case LED_BLE_OFF:
        ledState &= ~(UINT8_C(1) << BLE_BIT);
        APP_LED_OFF(BLE_LED);
        break;
    case LED_UWB_ON:
        ledState |= UINT8_C(1) << UWB_BIT;
        led_on(UWB_LED);
        break;
    case LED_UWB_OFF:
        ledState &= ~(UINT8_C(1) << UWB_BIT);
        APP_LED_OFF(UWB_LED);
        break;
    case LED_POWER_ON:
        ledState |= UINT8_C(1) << POWER_BIT;
        led_on(POWER_LED);
        break;
    case LED_POWER_OFF:
        ledState &= ~(UINT8_C(1) << POWER_BIT);
        APP_LED_OFF(POWER_LED);
        break;
    case LED_PWRAMP_ON:
        ledState |= UINT8_C(1) << PWRAMP_BIT;
        led_on(PWRAMP_LED);
        break;
    case LED_PWRAMP_OFF:
        ledState &= ~(UINT8_C(1) << PWRAMP_BIT);
        APP_LED_OFF(PWRAMP_LED);
        break;
    default:
        LOG_ERR("Attempted to update invalid LED: %d", update);
    }
}

void all_leds_off(void) {
    led_mode = led_mode_off;
    APP_LED_OFF(BLE_LED);
    APP_LED_OFF(UWB_LED);
    APP_LED_OFF(PWRAMP_LED);
    APP_LED_OFF(POWER_LED);
    dwt_setleds(DWT_LEDS_DISABLE);
    set_serial_led_master_state(false);
}

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

enum led_state get_ble_led_state(void) {
    return (ledState & BIT(BLE_BIT)) ? LED_BLE_ON : LED_BLE_OFF;
}

enum led_state get_uwb_led_state(void) {
    return (ledState & BIT(UWB_BIT)) ? LED_UWB_ON : LED_UWB_OFF;
}

enum led_state get_power_led_state(void) {
    return (ledState & BIT(POWER_BIT)) ? LED_POWER_ON : LED_POWER_OFF;
}

enum led_state get_unused_led_state(void) {
    return ledState & BIT(PWRAMP_BIT) ? LED_PWRAMP_ON : LED_PWRAMP_OFF;
}

bool are_leds_on(void) { return led_mode == led_mode_on; }
