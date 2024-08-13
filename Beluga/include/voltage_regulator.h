//
// Created by tom on 8/13/24.
//

#ifndef BELUGA_VOLTAGE_REGULATOR_H
#define BELUGA_VOLTAGE_REGULATOR_H

#include <stdbool.h>

enum voltage_level { VR_2V4, VR_3V3, VR_3V5 };

bool init_voltage_regulator(void);
bool update_voltage_level(enum voltage_level level);
enum voltage_level get_current_voltage(void);

#endif // BELUGA_VOLTAGE_REGULATOR_H
