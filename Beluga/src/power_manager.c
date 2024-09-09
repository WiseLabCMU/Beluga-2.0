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

static void configure_wake_source(void) {
    // TODO: Use accelerometer
}

void enter_deep_sleep(void) {
    sleep_dw1000();
    stop_ble();
    configure_wake_source();
    sys_poweroff();
}