//
// Created by tom on 12/10/24.
//

#ifndef BELUGA_SERIAL_LED_H
#define BELUGA_SERIAL_LED_H

enum serial_led_state { LED_START_TX, LED_STOP_TX, LED_START_RX, LED_STOP_RX };

int serial_leds_init(void);
int serial_leds_update_state(enum serial_led_state state);
int set_serial_led_master_state(bool enable);

#endif // BELUGA_SERIAL_LED_H
