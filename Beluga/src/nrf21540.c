//
// Created by tom on 8/13/24.
//

#include <assert.h>
#include <ble_app.h>
#include <nrf21540.h>
#include <settings.h>
#include <spi.h>
#include <stdbool.h>
#include <stdint.h>
#include <voltage_regulator.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#if defined(CONFIG_BELUGA_RANGE_EXTENSION)

#define NRF21540_NODE DT_NODELABEL(nrf_radio_fem)

#if DT_NODE_HAS_PROP(NRF21540_NODE, ant_sel_gpios)
#define ANTENNA_GPIO 1
#else
#define ANTENNA_GPIO 0
#endif

#if !DT_NODE_HAS_PROP(NRF21540_NODE, mode_gpios)
#error "nrf21540 must have mode gpio connected"
#endif

#if !DT_NODE_HAS_PROP(NRF21540_NODE, tx_en_gpios)
#error "nrf21540 must have TX gpio connected"
#endif

#if !DT_NODE_HAS_PROP(NRF21540_NODE, rx_en_gpios)
#error "nrf21540 must have RX gpio connected"
#endif

#if !DT_NODE_HAS_PROP(NRF21540_NODE, pdn_gpios)
#error "nrf21540 must have PDN gpio connected"
#endif

static struct nrf21540_gpios {
#if ANTENNA_GPIO
    const struct gpio_dt_spec ant_sel;
#endif
    const struct gpio_dt_spec power_mode;
} nrf21_gpios = {
#if ANTENNA_GPIO
        .ant_sel = GPIO_DT_SPEC_GET(NRF21540_NODE, ant_sel_gpios),
#endif
        .power_mode = GPIO_DT_SPEC_GET(NRF21540_NODE, mode_gpios),
};

static bool configure_nrf21_gpios(void) {
    int err;

#if ANTENNA_GPIO
    if (!device_is_ready(nrf21_gpios.ant_sel.port)) {
        printk("Antenna device was not ready\n");
        return false;
    }

    err = gpio_pin_configure_dt(&nrf21_gpios.ant_sel, GPIO_OUTPUT_INACTIVE);

    if (err) {
        printk("Antenna GPIO configure (%d)\n", err);
        return false;
    }
#endif

    if (!device_is_ready(nrf21_gpios.power_mode.port)) {
        printk("Power mode device was not ready\n");
        return false;
    }

    err = gpio_pin_configure_dt(&nrf21_gpios.power_mode, GPIO_OUTPUT_INACTIVE);

    if (err) {
        printk("Power mode GPIO configure (%d)\n", err);
        return false;
    }

    return true;
}

bool init_nrf21540(void) {
    bool retVal = true;
    bool state = save_and_disable_bluetooth();

    if (!configure_nrf21_gpios()) {
        retVal = false;
    }

    restore_bluetooth(state);

    printk("Front end module configured\n");

    return retVal;
}

bool select_ble_antenna(enum antenna_select antenna) {
    int err = 0;

#if ANTENNA_GPIO
    bool state = save_and_disable_bluetooth();
    switch (antenna) {
        case ANTENNA_1:
            err = gpio_pin_set_dt(&nrf21_gpios.ant_sel, 0);
            restore_bluetooth(state);
            break;
        case ANTENNA_2:
            err = gpio_pin_set_dt(&nrf21_gpios.ant_sel, 1);
            restore_bluetooth(state);
            break;
        default:
            printk("Invalid value\n");
            return false;
    }

    if (err) {
        printk("Failed to set GPIO: %d\n", err);
    }
#else
    printk("No antenna select found\n");
#endif

    return err == 0;
}

bool select_ble_gain(enum ble_gain gain) {
    int err = 0;

    bool state = save_and_disable_bluetooth();
    switch(gain) {
        case GAIN_0_DB:
            err = gpio_pin_set_dt(&nrf21_gpios.power_mode, 1);
            restore_bluetooth(state);
            break;
        case GAIN_20_DB:
            err = gpio_pin_set_dt(&nrf21_gpios.power_mode, 0);
            restore_bluetooth(state);
            break;
        default:
            printk("Invalid gain case\n");
            return false;
    }

    if (err) {
        printk("Unable to set gain: %d\n", err);
    }

    return err == 0;
}

#endif
