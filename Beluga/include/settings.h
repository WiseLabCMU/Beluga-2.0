//
// Created by tom on 7/11/24.
//

#ifndef BELUGA_SETTINGS_H
#define BELUGA_SETTINGS_H

#include <stdint.h>

enum beluga_setting {
    BELUGA_ID,
    BELUGA_BOOTMODE,
    BELUGA_POLL_RATE,
    BELUGA_UWB_CHANNEL,
    BELUGA_BLE_TIMEOUT,
    BELUGA_TX_POWER,
    BELUGA_STREAMMODE,
    BELUGA_TWR,
    BELUGA_LEDMODE,
    BELUGA_OUT_FORMAT,
    BELUGA_RANGE_EXTEND,
    BELUGA_UWB_DATA_RATE,
    BELUGA_UWB_PREAMBLE,
    BELUGA_UWB_PULSE_RATE,
    BELUGA_RESERVED
};

#define DEFAULT_ID_SETTING INT32_C(0)
#define DEFAULT_SETTING    INT32_C(-1)

void updateSetting(enum beluga_setting setting, int32_t value);
int32_t retrieveSetting(enum beluga_setting setting);
void resetBelugaSettings(void);
int initBelugaSettings(void);

#endif // BELUGA_SETTINGS_H
