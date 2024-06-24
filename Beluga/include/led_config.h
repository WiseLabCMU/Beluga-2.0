//
// Created by tom on 6/18/24.
//

#ifndef BELUGA_LED_CONFIG_H
#define BELUGA_LED_CONFIG_H

#include <dk_buttons_and_leds.h>

#define LED_SUPPORT
#define LED_SUPPORT_ENABLED        1
#define BLE_LED_CONFIG 1
#define APP_LED_CONFIG !BLE_LED_CONFIG

#define PERIPHERAL_ADVERTISING_LED DK_LED1
#define PERIPHERAL_CONNECTED_LED   DK_LED2
#define CENTRAL_SCANNING_LED       DK_LED3
#define CENTRAL_CONNECTED_LED      DK_LED4

#define UWB_LED DK_LED1
#define BLE_LED DK_LED2

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

#if BLE_LED_CONFIG == 1
#define BLE_LED_ON(LED) LED_ON(LED)
#define BLE_LED_OFF(LED) LED_OFF(LED)
#else
#define BLE_LED_ON(LED)                                                            \
    do {                                                                       \
    } while (0)
#define BLE_LED_OFF(LED) BLE_LED_ON(LED)
#endif

#if APP_LED_CONFIG == 1
#define APP_LED_ON(LED) LED_ON(LED)
#define APP_LED_OFF(LED) LED_OFF(LED)
#else
#define APP_LED_ON(LED)                                                            \
    do {                                                                       \
    } while (0)
#define APP_LED_OFF(LED) APP_LED_ON(LED)
#endif

#endif // BELUGA_LED_CONFIG_H
