//
// Created by tom on 6/18/24.
//

#ifndef BELUGA_LED_CONFIG_H
#define BELUGA_LED_CONFIG_H

#if defined(CONFIG_DK_LIBRARY) && defined(CONFIG_BELUGA_LEDS)
#define LED_SUPPORT_ENABLED 1
#else
#define LED_SUPPORT_ENABLED 0
#endif

#if defined(CONFIG_BELUGA_BLE_LEDS)
#define BLE_LED_CONFIG 1
#else
#define BLE_LED_CONFIG 0
#endif

#if LED_SUPPORT_ENABLED == 1
#include <dk_buttons_and_leds.h>
#define LED_INIT                                                               \
    do {                                                                       \
        int _ledErr = dk_leds_init();                                          \
        if (_ledErr) {                                                         \
            printk("LEDs init failed (err %d)\n", _ledErr);                    \
            return 0;                                                          \
        }                                                                      \
    } while (0)
#else
#define LED_INIT                                                               \
    do {                                                                       \
    } while (0)

#undef DK_LED1
#define DK_LED1 1

#undef DK_LED2
#define DK_LED2 2

#undef DK_LED3
#define DK_LED3 3

#undef DK_LED4
#define DK_LED4 4
#endif

#define APP_LED_CONFIG             !BLE_LED_CONFIG

#define PERIPHERAL_ADVERTISING_LED DK_LED1
#define PERIPHERAL_CONNECTED_LED   DK_LED2
#define CENTRAL_SCANNING_LED       DK_LED3
#define CENTRAL_CONNECTED_LED      DK_LED4

#define UWB_LED                    DK_LED1
#define BLE_LED                    DK_LED2
#define UNUSED_LED                 DK_LED3
#define POWER_LED                  DK_LED4

#if LED_SUPPORT_ENABLED == 1
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
#define BLE_LED_ON(LED)  LED_ON(LED)
#define BLE_LED_OFF(LED) LED_OFF(LED)
#else
#define BLE_LED_ON(LED)                                                        \
    do {                                                                       \
    } while (0)
#define BLE_LED_OFF(LED) BLE_LED_ON(LED)
#endif

#if APP_LED_CONFIG == 1
#define APP_LED_ON(LED)  LED_ON(LED)
#define APP_LED_OFF(LED) LED_OFF(LED)
#else
#define APP_LED_ON(LED)                                                        \
    do {                                                                       \
    } while (0)
#define APP_LED_OFF(LED) APP_LED_ON(LED)
#endif

#endif // BELUGA_LED_CONFIG_H
