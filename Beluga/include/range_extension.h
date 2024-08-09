//
// Created by tom on 8/5/24.
//

#ifndef BELUGA_RANGE_EXTENSION_H
#define BELUGA_RANGE_EXTENSION_H

#include <stdbool.h>

enum antenna_select { ANTENNA_1, ANTENNA_2 };

bool init_range_extension(void);
bool enable_range_extension(void);
bool disable_range_extension(void);
bool select_antenna(enum antenna_select ant);

#endif // BELUGA_RANGE_EXTENSION_H
