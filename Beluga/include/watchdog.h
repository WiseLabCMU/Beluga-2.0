//
// Created by tom on 7/8/24.
//

#ifndef BELUGA_WATCHDOG_H
#define BELUGA_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

struct task_wdt_attr {
    int32_t id;
    bool starving;
    uint32_t period;
};

int configure_watchdog_timer(void);
int spawn_task_watchdog(struct task_wdt_attr *attr);
void let_the_dog_starve(struct task_wdt_attr *attr);
void watchdog_red_rocket(struct task_wdt_attr *attr);

#endif // BELUGA_WATCHDOG_H
