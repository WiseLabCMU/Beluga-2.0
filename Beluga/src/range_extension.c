//
// Created by tom on 8/5/24.
//

#include <app_leds.h>
#include <range_extension.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(range_ext_logger, CONFIG_RANGE_EXTENSION_LOG_LEVEL);

#if defined(CONFIG_BELUGA_RANGE_EXTENSION)
#include <deca_device_api.h>
#include <nrf21540.h>

bool init_range_extension(void) {
    LOG_INF("Initializing range extension");
    return init_nrf21540();
}

bool enable_range_extension(bool command) {
    ARG_UNUSED(command);
    bool retVal = select_ble_gain(GAIN_20_DB);
    if (retVal) {
        dwt_setlnapamode(0, 1);
    }
    if (retVal) {
        update_led_state(LED_PWRAMP_ON);
    }
    return retVal;
}

bool disable_range_extension(bool command) {
    ARG_UNUSED(command);
    bool retVal = select_ble_gain(GAIN_0_DB);
    if (retVal) {
        dwt_setlnapamode(0, 1);
    }
    if (retVal) {
        update_led_state(LED_PWRAMP_OFF);
    }
    return retVal;
}

bool select_antenna(int32_t ant) {
    bool retVal = false;
    switch (ant) {
    case 1:
        retVal = select_ble_antenna(ANTENNA_1);
        break;
    case 2:
        retVal = select_ble_antenna(ANTENNA_2);
        break;
    default:
        printf("Invalid antenna value\r\n");
        break;
    }

    return retVal;
}

#else
bool init_range_extension(void) {
    LOG_INF("Range extension disabled");
    return true;
}

bool enable_range_extension(bool command) {
    if (command) {
        printf("Not implemented\r\n");
    }
    return false;
}

bool disable_range_extension(bool command) {
    if (command) {
        printf("Not implemented\r\n");
    }
    return false;
}

bool select_antenna(int32_t ant) {
    printf("Not implemented\r\n");
    return false;
}
#endif
