//
// Created by tom on 11/22/24.
//

#ifndef BELUGA_RESPONDER_H
#define BELUGA_RESPONDER_H

#include <zephyr/kernel.h>

int set_responder_pan_id(uint16_t id);
int set_responder_id(uint16_t id);
int ds_resp_run(uint16_t *id, uint32_t *logic_clk);
int ss_resp_run(uint16_t *id, uint32_t *logic_clk);

extern struct k_sem k_sus_resp;

#endif // BELUGA_RESPONDER_H
