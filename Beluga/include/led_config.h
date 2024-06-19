//
// Created by tom on 6/18/24.
//

#ifndef BELUGA_LED_CONFIG_H
#define BELUGA_LED_CONFIG_H

#include <dk_buttons_and_leds.h>

#define LED_SUPPORT
#define LED_SUPPORT_ENABLED        1

#define PERIPHERAL_ADVERTISING_LED DK_LED1
#define PERIPHERAL_CONNECTED_LED   DK_LED2
#define CENTRAL_SCANNING_LED       DK_LED3
#define CENTRAL_CONNECTED_LED      DK_LED4

#if defined(LED_SUPPORT) && LED_SUPPORT_ENABLED == 1
#define LED_ON(LED)                                                            \
    do {                                                                       \
        dk_set_led_on(LED);                                                    \
    } while (0)
#define LED_OFF(LED)                                                           \
    do {                                                                       \
        dk_set_led_off(LED);                                                   \
    } while (0)
#else
#define LED_ON(LED)                                                            \
    do {                                                                       \
    } while (0)
#define LED_OFF(LED) LED_ON(LED)
#endif

#endif // BELUGA_LED_CONFIG_H
