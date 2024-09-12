//
// Created by tom on 7/10/24.
//

#ifndef BELUGA_APP_LEDS_H
#define BELUGA_APP_LEDS_H

#include <stdbool.h>

enum led_state {
    LED_BLE_ON,
    LED_BLE_OFF,
    LED_UWB_ON,
    LED_UWB_OFF,
    LED_POWER_ON,
    LED_POWER_OFF,
    LED_PWRAMP_ON,
    LED_PWRAMP_OFF
};

void update_led_state(enum led_state update);
void all_leds_off(void);
void restore_led_states(void);
enum led_state get_ble_led_state(void);
enum led_state get_uwb_led_state(void);
enum led_state get_power_led_state(void);
enum led_state get_pwramp_led_state(void);
bool are_leds_on(void);

#endif // BELUGA_APP_LEDS_H
