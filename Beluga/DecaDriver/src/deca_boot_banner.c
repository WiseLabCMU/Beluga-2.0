/**
 * @file deca_boot_banner.c
 *
 * @brief Displays the DW1000 driver version before start
 *
 * @date 12/13/2024
 *
 * @author Tom Schmitz
 */

#include <deca_version.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

int dw1000_boot_banner(void) {
#if IS_ENABLED(CONFIG_UWB_BOOT_BANNER)
    printk("*** " DW1000_DEVICE_DRIVER_VER_STRING " ***\n");
#endif
    return 0;
}

SYS_INIT(dw1000_boot_banner, POST_KERNEL, 0);