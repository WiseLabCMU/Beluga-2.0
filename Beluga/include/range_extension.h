//
// Created by tom on 8/5/24.
//

#ifndef BELUGA_RANGE_EXTENSION_H
#define BELUGA_RANGE_EXTENSION_H

#include <stdbool.h>
#include <stdint.h>

enum ble_power_mode {
    POWER_MODE_BYPASS, ///< RF Bypass
    POWER_MODE_LOW,    ///< Amplify a little
    POWER_MODE_HIGH    ///< Tell FCC to fuck off
};

int init_range_extension(void);
int update_power_mode(enum ble_power_mode mode);
int select_antenna(int32_t ant);
int update_fem_shutdown_state(bool shutdown);

#endif // BELUGA_RANGE_EXTENSION_H
