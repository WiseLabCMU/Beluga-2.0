/*----------------------------------------------------------------------------
 *  @file    init_main.h
 *  @brief   Double-sided and Single-sided two-way ranging (DS/SS TWR) initiator
 * code -- Header file
 *
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#ifndef DECA_INIT_MAIN_H
#define DECA_INIT_MAIN_H

#include "deca_types.h"
#include <stdbool.h>
#include <stdint.h>

extern int debug_print;

double ds_init_run(uint8 id);
double ss_init_run(uint8 id);

#endif // DECA_INIT_MAIN_H
