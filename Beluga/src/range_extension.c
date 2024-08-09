//
// Created by tom on 8/5/24.
//

#include <range_extension.h>
#include <stdio.h>
#include <zephyr/kernel.h>

#if defined(CONFIG_FEM_AL_LIB) && defined(CONFIG_BELUGA_RANGE_EXTENSION)
#include <fem_al/fem_al.h>
#include <deca_device_api.h>
#include <nrf.h>

const uint32_t nrf_power_value[] = {
#if defined(RADIO_TXPOWER_TXPOWER_Neg40dBm)
        RADIO_TXPOWER_TXPOWER_Neg40dBm,
#endif /* RADIO_TXPOWER_TXPOWER_Neg40dBm */
        RADIO_TXPOWER_TXPOWER_Neg30dBm,
        RADIO_TXPOWER_TXPOWER_Neg20dBm,
        RADIO_TXPOWER_TXPOWER_Neg16dBm,
        RADIO_TXPOWER_TXPOWER_Neg12dBm,
        RADIO_TXPOWER_TXPOWER_Neg8dBm,
#if defined(RADIO_TXPOWER_TXPOWER_Neg7dBm)
        RADIO_TXPOWER_TXPOWER_Neg7dBm,
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg7dBm) */
#if defined(RADIO_TXPOWER_TXPOWER_Neg6dBm)
        RADIO_TXPOWER_TXPOWER_Neg6dBm,
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg6dBm) */
#if defined(RADIO_TXPOWER_TXPOWER_Neg5dBm)
        RADIO_TXPOWER_TXPOWER_Neg5dBm,
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg5dBm) */
        RADIO_TXPOWER_TXPOWER_Neg4dBm,
#if defined(RADIO_TXPOWER_TXPOWER_Neg3dBm)
        RADIO_TXPOWER_TXPOWER_Neg3dBm,
#endif /* defined (RADIO_TXPOWER_TXPOWER_Neg3dBm) */
#if defined(RADIO_TXPOWER_TXPOWER_Neg2dBm)
        RADIO_TXPOWER_TXPOWER_Neg2dBm,
#endif /* defined (RADIO_TXPOWER_TXPOWER_Neg2dBm) */
#if defined(RADIO_TXPOWER_TXPOWER_Neg1dBm)
        RADIO_TXPOWER_TXPOWER_Neg1dBm,
#endif /* defined (RADIO_TXPOWER_TXPOWER_Neg1dBm) */
        RADIO_TXPOWER_TXPOWER_0dBm,
#if defined(RADIO_TXPOWER_TXPOWER_Pos1dBm)
        RADIO_TXPOWER_TXPOWER_Pos1dBm,
#endif /* RADIO_TXPOWER_TXPOWER_Pos1dBm */
#if defined(RADIO_TXPOWER_TXPOWER_Pos2dBm)
        RADIO_TXPOWER_TXPOWER_Pos2dBm,
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos2dBm) */
#if defined(RADIO_TXPOWER_TXPOWER_Pos3dBm)
        RADIO_TXPOWER_TXPOWER_Pos3dBm,
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos3dBm) */
#if defined(RADIO_TXPOWER_TXPOWER_Pos4dBm)
        RADIO_TXPOWER_TXPOWER_Pos4dBm,
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos4dBm) */
#if defined(RADIO_TXPOWER_TXPOWER_Pos5dBm)
        RADIO_TXPOWER_TXPOWER_Pos5dBm,
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos5dBm) */
#if defined(RADIO_TXPOWER_TXPOWER_Pos6dBm)
        RADIO_TXPOWER_TXPOWER_Pos6dBm,
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos6dBm) */
#if defined(RADIO_TXPOWER_TXPOWER_Pos7dBm)
        RADIO_TXPOWER_TXPOWER_Pos7dBm,
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos7dBm) */
#if defined(RADIO_TXPOWER_TXPOWER_Pos8dBm)
        RADIO_TXPOWER_TXPOWER_Pos8dBm
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos8dBm) */
};

// TODO

void init_range_extension(void) {

}

void enable_range_extension(void) {

}

void disable_range_extension(void) {

}

#elif defined(CONFIG_BELUGA_RANGE_EXTENSION)
#include <zephyr/drivers/gpio.h>

DT_NODELABEL(nrf_radio_fem)

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
