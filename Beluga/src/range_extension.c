//
// Created by tom on 8/5/24.
//

#include <range_extension.h>
#include <stdio.h>
#include <zephyr/kernel.h>

#if defined(CONFIG_FEM_AL_LIB) && defined(CONFIG_BELUGA_RANGE_EXTENSION)
#include <deca_device_api.h>
#include <fem_al/fem_al.h>
#include <nrf.h>

// TODO

void init_range_extension(void) {}

void enable_range_extension(void) {}

void disable_range_extension(void) {}

#elif defined(CONFIG_BELUGA_RANGE_EXTENSION)
// Only GPIO support

#include <ble_app.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define NRF21540_NODE DT_NODELABEL(nrf_radio_fem)

#if DT_NODE_HAS_PROP(NRF21540_NODE, ant_sel_gpios)
#define ANTENNA_GPIO_DEFINED 1
#else
#define ANTENNA_GPIO_DEFINED 0
#endif // DT_NODE_HAS_PROP(NRF21540_NODE, ant_sel_gpios)

#if DT_NODE_HAS_PROP(NRF21540_NODE, mode_gpios)
#define MODE_GPIO_DEFINED 1
#else
#define MODE_GPIO_DEFINED 0
#endif // DT_NODE_HAS_PROP(NRF21540_NODE, mode_gpios)

static struct gpio_range_extension {
#if ANTENNA_GPIO_DEFINED
    const struct gpio_dt_spec ant_sel;
#endif
#if MODE_GPIO_DEFINED
    const struct gpio_dt_spec power_mode;
#endif
} range_extension_gpio = {
#if ANTENNA_GPIO_DEFINED
    .ant_sel = GPIO_DT_SPEC_GET(NRF21540_NODE, ant_sel_gpios),
#endif
#if MODE_GPIO_DEFINED
    .power_mode = GPIO_DT_SPEC_GET(NRF21540_NODE, mode_gpios),
#endif
};

bool init_range_extension(void) {
    int err = 0;

#if ANTENNA_GPIO_DEFINED
    if (!device_is_ready(range_extension_gpio.ant_sel.port)) {
        printk("Antenna device not ready\n");
        return false;
    }

    err = gpio_pin_configure_dt(&range_extension_gpio.ant_sel,
                                GPIO_OUTPUT_INACTIVE);

    if (err) {
        printk("Antenna GPIO configure (%d)", err);
        return false;
    }
#endif

#if MODE_GPIO_DEFINED
    if (!device_is_ready(range_extension_gpio.power_mode.port)) {
        printk("Antenna device not ready\n");
        return false;
    }

    err = gpio_pin_configure_dt(&range_extension_gpio.power_mode,
                                GPIO_OUTPUT_INACTIVE);

    if (err) {
        printk("Power mode GPIO configure (%d)", err);
        return false;
    }
#endif

    printk("Simple range extension init\n");
    return true;
}

bool enable_range_extension(void) {
#if MODE_GPIO_DEFINED
    if (check_ble_enabled()) {
        printf(
            "Please disable bluetooth before enabling the power amplifier\r\n");
        return false;
    }
    int err;
    if ((err = gpio_pin_set_dt(&range_extension_gpio.power_mode, 1)) != 0) {
        printk("Unable to update range (%d)\n", err);
        printf("Unable to update range\r\n");
        return false;
    }
    return true;
#else
    printf("Not implemented\r\n");
    return false;
#endif
}

bool disable_range_extension(void) {
#if MODE_GPIO_DEFINED
    if (check_ble_enabled()) {
        printf("Please disable bluetooth before disabling the power "
               "amplifier\r\n");
        return false;
    }
    int err;
    if ((err = gpio_pin_set_dt(&range_extension_gpio.power_mode, 0)) != 0) {
        printk("Unable to update range (%d)\n", err);
        printf("Unable to update range\r\n");
        return false;
    }
    return true;
#else
    printf("Not implemented\r\n");
    return false;
#endif
}

bool select_antenna(enum antenna_select ant) {
#if ANTENNA_GPIO_DEFINED
    int err = 0;

    switch (ant) {
    case ANTENNA_1:
        err = gpio_pin_set_dt(&range_extension_gpio.ant_sel, 0);
        break;
    case ANTENNA_2:
        err = gpio_pin_set_dt(&range_extension_gpio.ant_sel, 1);
        break;
    default:
        printk("Invalid antenna input");
        break;
    }

    if (err) {
        printk("Antenna select error (%d)\n", err);
        printf("A problem occurred\r\n");
        return false;
    }
    return true;
#else
    printf("Not implemented\n");
    return false;
#endif
}

#else
void init_range_extension(void) { printk("Range extension disabled\n"); }

void enable_range_extension(void) { printf("Not implemented\r\n"); }

void disable_range_extension(void) { printf("Not implemented\r\n"); }
#endif
