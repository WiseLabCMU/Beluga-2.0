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

#if LED_SUPPORT_ENABLED == 0
#define DK_LED1 1
#define DK_LED2 2
#define DK_LED3 3
#define DK_LED4 4
#else
#include <dk_buttons_and_leds.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#endif

#define APP_LED_CONFIG             !BLE_LED_CONFIG

#define PERIPHERAL_ADVERTISING_LED DK_LED1
#define PERIPHERAL_CONNECTED_LED   DK_LED2
#define CENTRAL_SCANNING_LED       DK_LED3
#define CENTRAL_CONNECTED_LED      DK_LED4

#define UWB_LED                    DK_LED3
#define BLE_LED                    DK_LED4
#define PWRAMP_LED                 DK_LED1
#define POWER_LED                  DK_LED2

#if LED_SUPPORT_ENABLED == 1

#define LED_EARLY_EXIT_VOID 0
#define LED_EARLY_EXIT_INT  1

#define _LED_ACTION_NO_RET(_func, LED)                                         \
    do {                                                                       \
        int err = _func(LED);                                                  \
        if (err) {                                                             \
            LOG_ERR("Unable to update LED using " #_func "(). Returned (%d)",  \
                    err);                                                      \
        }                                                                      \
    } while (0)

#define _LED_ACTION_RET_VOID(_func, LED)                                       \
    do {                                                                       \
        int err = _func(LED);                                                  \
        if (err) {                                                             \
            LOG_ERR("Unable to update LED using " #_func "(). Returned (%d)",  \
                    err);                                                      \
            return;                                                            \
        }                                                                      \
    } while (0)

#define _LED_ACTION_RET_ERR(_func, LED)                                        \
    do {                                                                       \
        int err = _func(LED);                                                  \
        if (err) {                                                             \
            LOG_ERR("Unable to update LED using " #_func "(). Returned (%d)",  \
                    err);                                                      \
            return err;                                                        \
        }                                                                      \
    } while (0)

#define _LED_ACTION_RET(_func, LED, arg)                                       \
    COND_CODE_1(arg, (_LED_ACTION_RET_ERR(_func, LED)),                        \
                (_LED_ACTION_RET_VOID(_func, LED)))

#define _LED_GENERATE_CODE(LED, _func, ...)                                    \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (_LED_ACTION_NO_RET(_func, LED)),       \
                (_LED_ACTION_RET(_func, LED, GET_ARG_N(1, __VA_ARGS__))))

#define LED_ON(LED, ...)  _LED_GENERATE_CODE(LED, dk_set_led_on, __VA_ARGS__)

#define LED_OFF(LED, ...) _LED_GENERATE_CODE(LED, dk_set_led_off, __VA_ARGS__)
#else
#define LED_ON(LED, ...)  (void)0
#define LED_OFF(LED, ...) LED_ON(LED)
#endif

#if LED_SUPPORT_ENABLED == 1
#define LED_INIT                                                               \
    do {                                                                       \
        int _ledErr = dk_leds_init();                                          \
        if (_ledErr) {                                                         \
            LOG_ERR("LEDs init failed (err %d)\n", _ledErr);                   \
            return 0;                                                          \
        }                                                                      \
        LED_OFF(DK_LED1, LED_EARLY_EXIT_INT);                                  \
        LED_OFF(DK_LED2, LED_EARLY_EXIT_INT);                                  \
        LED_OFF(DK_LED2, LED_EARLY_EXIT_INT);                                  \
        LED_OFF(DK_LED3, LED_EARLY_EXIT_INT);                                  \
        LED_OFF(DK_LED4, LED_EARLY_EXIT_INT);                                  \
        LOG_INF("LEDs initialized");                                           \
    } while (0)
#else
#define LED_INIT                                                               \
    do {                                                                       \
        LOG_INF("LEDs disabled");                                              \
    } while (0)
#endif

#if BLE_LED_CONFIG == 1
#define BLE_LED_ON(LED, ...)  LED_ON(LED, __VA_ARGS__)
#define BLE_LED_OFF(LED, ...) LED_OFF(LED, __VA_ARGS__)
#else
#define BLE_LED_ON(LED) (void)0
#define BLE_LED_OFF(LED) BLE_LED_ON(LED)
#endif

#if APP_LED_CONFIG == 1
#define APP_LED_ON(LED, ...)  LED_ON(LED, __VA_ARGS__)
#define APP_LED_OFF(LED, ...) LED_OFF(LED, __VA_ARGS__)
#else
#define APP_LED_ON(LED) (void)0
#define APP_LED_OFF(LED) APP_LED_ON(LED)
#endif

#endif // BELUGA_LED_CONFIG_H
