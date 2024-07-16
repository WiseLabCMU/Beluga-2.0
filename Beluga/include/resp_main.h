/*----------------------------------------------------------------------------
 *  @file    resp_main.h
 *  @brief   Double-sided and Single-sided two-way ranging (DS/SS TWR) responder
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

/* Declaration of static functions. */

#ifndef DECA_RESP_MAIN_H
#define DECA_RESP_MAIN_H

#include <zephyr/kernel.h>

int ds_resp_run(void);
int ss_resp_run(void);

extern struct k_sem k_sus_resp;

#endif // DECA_RESP_MAIN_H
