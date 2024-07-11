//
// Created by tom on 7/10/24.
//

#include <led_config.h>
#include <app_leds.h>
#include <stdint.h>
#include <assert.h>
#include <stdbool.h>
#include <zephyr/sys/util.h>
#include <deca_device_api.h>

#define BLE_BIT 0
#define UWB_BIT 1
#define UNUSED_BIT 2
#define POWER_BIT 3

#define led_mode_om true
#define led_mode_off false

static uint8_t ledState = 0;
static bool led_mode = led_mode_om;

static void led_on(uint32_t led) {
    if (led_mode) {
        APP_LED_ON(led);
    }
}

void update_led_state(enum led_state update) {
    switch(update) {
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
        case LED_UNUSED_ON:
            ledState |= UINT8_C(1) << UNUSED_BIT;
            led_on(UNUSED_LED);
            break;
        case LED_UNUSED_OFF:
            ledState &= ~(UINT8_C(1) << UNUSED_BIT);
            APP_LED_OFF(UNUSED_LED);
            break;
        default:
            assert(false);
    }
}

void all_leds_off(void) {
    led_mode = led_mode_off;
    APP_LED_OFF(BLE_LED);
    APP_LED_OFF(UWB_LED);
    APP_LED_OFF(UNUSED_LED);
    APP_LED_OFF(POWER_LED);
    //dwt_setleds(DWT_LEDS_DISABLE);
}

void restore_led_states(void) {
    all_leds_off();

    if (ledState & BIT(BLE_BIT)) {
        APP_LED_ON(BLE_LED);
    }

    if (ledState & BIT(UWB_BIT)) {
        APP_LED_ON(UWB_LED);
    }

    if (ledState & BIT(UNUSED_BIT)) {
        APP_LED_ON(UNUSED_LED);
    }

    if (ledState & BIT(POWER_BIT)) {
        APP_LED_ON(POWER_LED);
    }
    led_mode = led_mode_om;
    //dwt_setleds(DWT_LEDS_ENABLE);
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
    return ledState & BIT(UNUSED_BIT) ? LED_UNUSED_ON : LED_UNUSED_OFF;
}
