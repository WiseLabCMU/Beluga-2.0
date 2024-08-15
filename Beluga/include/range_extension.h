//
// Created by tom on 8/5/24.
//

#ifndef BELUGA_RANGE_EXTENSION_H
#define BELUGA_RANGE_EXTENSION_H

#include <stdbool.h>
#include <stdint.h>

bool init_range_extension(void);
bool enable_range_extension(void);
bool disable_range_extension(void);
bool select_antenna(int32_t ant);

#endif // BELUGA_RANGE_EXTENSION_H
