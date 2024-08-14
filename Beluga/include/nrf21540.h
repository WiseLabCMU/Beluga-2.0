//
// Created by tom on 8/13/24.
//

#ifndef BELUGA_NRF21540_H
#define BELUGA_NRF21540_H

enum antenna_select {
    ANTENNA_1,
    ANTENNA_2
};

enum ble_gain {
    GAIN_0_DB,
    GAIN_20_DB
};

bool init_nrf21540(void);
bool select_ble_antenna(enum antenna_select antenna);
bool select_ble_gain(enum ble_gain gain);

#endif // BELUGA_NRF21540_H
