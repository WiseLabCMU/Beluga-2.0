//
// Created by tom on 8/5/24.
//

#include <range_extension.h>
#include <stdio.h>
#include <zephyr/kernel.h>

#ifdef CONFIG_FEM_AL_LIB
#include <fem_al/fem_al.h>
#include <deca_device_api.h>

// TODO

void init_range_extension(void) {

}

void enable_range_extension(void) {

}

void disable_range_extension(void) {

}

#else
void init_range_extension(void) {
    printk("Range extension disabled\n");
}

void enable_range_extension(void) {
    printf("Not implemented\r\n");
}

void disable_range_extension(void) {
    printf("Not implemented\r\n");
}
#endif
