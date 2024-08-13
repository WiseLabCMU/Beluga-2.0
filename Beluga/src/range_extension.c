//
// Created by tom on 8/5/24.
//

#include <range_extension.h>
#include <stdio.h>
#include <zephyr/kernel.h>

#if defined(CONFIG_BELUGA_RANGE_EXTENSION)
#include <nrf21540.h>

bool init_range_extension(void) {
    return init_nrf21540();
}

bool enable_range_extension(void) {
    printf("Not implemented\r\n");
    return false;
}

bool disable_range_extension(void) {
    printf("Not implemented\r\n");
    return false;
}

bool select_antenna(enum antenna_select ant) {
    printf("Not implemented\n");
    return false;
}

#else
bool init_range_extension(void) {
    printk("Range extension disabled\n");
    return true;
}

bool enable_range_extension(void) {
    printf("Not implemented\r\n");
    return false;
}

bool disable_range_extension(void) {
    printf("Not implemented\r\n");
    return false;
}

bool select_antenna(enum antenna_select ant) {
    printf("Not implemented\r\n");
    return false;
}
#endif
