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
#include <stdio.h>
#include <voltage_regulator.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define NRF21540_NODE DT_NODELABEL(nrf_radio_fem)

LOG_MODULE_REGISTER(nrf21540_logger, CONFIG_NRF21540_MODULE_LOG_LEVEL);

#if defined(CONFIG_BELUGA_RANGE_EXTENSION) && DT_NODE_EXISTS(NRF21540_NODE) && \
    DT_NODE_HAS_STATUS(NRF21540_NODE, okay)

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
        LOG_ERR("Antenna device was not ready");
        return false;
    }

    err = gpio_pin_configure_dt(&nrf21_gpios.ant_sel, GPIO_OUTPUT_INACTIVE);

    if (err) {
        LOG_ERR("Antenna GPIO configure (%d)", err);
        return false;
    }
#endif

    if (!device_is_ready(nrf21_gpios.power_mode.port)) {
        LOG_ERR("Power mode device was not ready");
        return false;
    }

    err = gpio_pin_configure_dt(&nrf21_gpios.power_mode, GPIO_OUTPUT_ACTIVE);

    if (err) {
        LOG_ERR("Power mode GPIO configure (%d)", err);
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

    LOG_INF("Front end module configured");

    return retVal;
}

bool select_ble_antenna(enum antenna_select antenna) {
    int err = 1;

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
        LOG_ERR("Invalid value");
        return false;
    }

    if (err) {
        LOG_ERR("Failed to set GPIO: %d", err);
    }
#else
    printf("Not implemented\r\n");
#endif

    return err == 0;
}

bool select_ble_gain(enum ble_gain gain) {
    int err = 0;

    bool state = save_and_disable_bluetooth();
    switch (gain) {
    case GAIN_0_DB:
        err = gpio_pin_set_dt(&nrf21_gpios.power_mode, 1);
        restore_bluetooth(state);
        break;
    case GAIN_20_DB:
        err = gpio_pin_set_dt(&nrf21_gpios.power_mode, 0);
        restore_bluetooth(state);
        break;
    default:
        LOG_ERR("Invalid gain case\n");
        return false;
    }

    if (err) {
        LOG_ERR("Unable to set gain: %d\n", err);
    }

    return err == 0;
}

#else
bool init_nrf21540(void) {
    LOG_INF("nrf21540 disabled\r\n");
    return true;
}

bool select_ble_antenna(enum antenna_select antenna) {
    printf("Not implemented\r\n");
    return false;
}

bool select_ble_gain(enum ble_gain gain) {
    printf("Not Implemented\r\n");
    return false;
}

#endif
