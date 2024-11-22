//
// Created by tom on 11/22/24.
//

#ifndef BELUGA_RESPONDER_H
#define BELUGA_RESPONDER_H

#include <zephyr/kernel.h>

int ds_resp_run(void);
int ss_resp_run(void);

extern struct k_sem k_sus_resp;

#endif // BELUGA_RESPONDER_H
