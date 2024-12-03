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

bool init_range_extension(void);
bool update_power_mode(enum ble_power_mode mode);
bool select_antenna(int32_t ant);
bool update_fem_shutdown_state(bool shutdown);

#endif // BELUGA_RANGE_EXTENSION_H
