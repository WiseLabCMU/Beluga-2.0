//
// Created by tom on 7/9/24.
//

#ifndef BELUGA_RANGING_H
#define BELUGA_RANGING_H

#include <deca_regs.h>

enum pgdelay_ch {
    ch1 = TC_PGDELAY_CH1,
    ch2 = TC_PGDELAY_CH2,
    ch3 = TC_PGDELAY_CH3,
    ch4 = TC_PGDELAY_CH4,
    ch5 = TC_PGDELAY_CH5,
    ch7 = TC_PGDELAY_CH7,
};

void set_twr_mode(bool value);
bool get_twr_mode(void);

#endif // BELUGA_RANGING_H
