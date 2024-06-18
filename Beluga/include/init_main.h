/*----------------------------------------------------------------------------
*  @file    init_main.h
*  @brief   Double-sided and Single-sided two-way ranging (DS/SS TWR) initiator code -- Header file
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

#include "FreeRTOS.h"
#include "semphr.h"
#include "deca_types.h"

extern SemaphoreHandle_t rxSemaphore, txSemaphore, sus_resp, sus_init;
extern int debug_print;
extern uint16_t NODE_UUID;

double ds_init_run(uint8 id);
double ss_init_run(uint8 id);

void set_initiator_freq(int value);
bool set_uwb_pgdelay(uint32_t channel);
void set_time_out(int timeout);
bool set_uwb_tx_power(uint32_t tx_power);
bool set_streaming_mode(uint32_t mode);
bool set_twr_mode(uint32_t mode);
