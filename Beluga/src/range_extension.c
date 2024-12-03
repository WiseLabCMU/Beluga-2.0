//
// Created by tom on 8/5/24.
//

#include <app_leds.h>
#include <range_extension.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(range_ext_logger, CONFIG_RANGE_EXTENSION_LOG_LEVEL);

#define SKY_GPIOS DT_NODELABEL(sky_fem_gpios)

#if defined(CONFIG_BELUGA_RANGE_EXTENSION) && DT_NODE_EXISTS(SKY_GPIOS)
#include <ble_app.h>
#include <deca_device_api.h>
#include <zephyr/drivers/gpio.h>

#if DT_NODE_HAS_PROP(SKY_GPIOS, ant_sel_gpios)
#define ANTENNA_SELECT 1
#else
#define ANTENNA_SELECT 0
#endif

#if !DT_NODE_HAS_PROP(SKY_GPIOS, csd_gpios)
#error "Shutdown gpio property (csd-gpios) is missing"
#endif

#if DT_NODE_HAS_PROP(SKY_GPIOS, cps_gpios)
#define RF_BYPASS 1
#else
#define RF_BYPASS 0
#endif

#if DT_NODE_HAS_PROP(SKY_GPIOS, chl_gpios)
#define HIGHLOW_POWER 1
#else
#define HIGHLOW_POWER 0
#endif

static struct fem_gpios {
#if ANTENNA_SELECT
    const struct gpio_dt_spec ant_sel;
#endif
    const struct gpio_dt_spec shutdown;
#if RF_BYPASS
    const struct gpio_dt_spec bypass;
#endif
#if HIGHLOW_POWER
    const struct gpio_dt_spec power;
#endif
} _fem_gpios = {
#if ANTENNA_SELECT
    .ant_sel = GPIO_DT_SPEC_GET(SKY_GPIOS, ant_sel_gpios),
#endif
    .shutdown = GPIO_DT_SPEC_GET(SKY_GPIOS, csd_gpios),
#if RF_BYPASS
    .bypass = GPIO_DT_SPEC_GET(SKY_GPIOS, cps_gpios),
#endif
#if HIGHLOW_POWER
    .power = GPIO_DT_SPEC_GET(SKY_GPIOS, chl_gpios),
#endif
};

#define INIT_FEM_PIN(container, attr, config)                                  \
    do {                                                                       \
        int err;                                                               \
        if (!device_is_ready(container.attr.port)) {                           \
            printk(#attr " was not ready\n");                                  \
            return false;                                                      \
        }                                                                      \
        err = gpio_pin_configure_dt(&container.attr, config);                  \
        if (err) {                                                             \
            printk(#attr " configure (%d)\n", err);                            \
            return false;                                                      \
        }                                                                      \
    } while (0)

bool init_range_extension(void) {
#if ANTENNA_SELECT
    INIT_FEM_PIN(_fem_gpios, ant_sel, GPIO_OUTPUT_INACTIVE);
#endif

    INIT_FEM_PIN(_fem_gpios, shutdown, GPIO_OUTPUT_LOW);

#if RF_BYPASS
    INIT_FEM_PIN(_fem_gpios, bypass, GPIO_OUTPUT_HIGH);
#endif

#if HIGHLOW_POWER
    INIT_FEM_PIN(_fem_gpios, power, GPIO_OUTPUT_LOW);
#endif

    return true;
}

bool update_power_mode(enum ble_power_mode mode) {
    bool retVal = true;
    int err;
    int uwb_pa = 0;
    bool ble_state = save_and_disable_bluetooth();

    switch (mode) {
    case POWER_MODE_BYPASS: {
#if RF_BYPASS
        err = gpio_pin_set_dt(&_fem_gpios.bypass, 1);

        if (err) {
            printk("Failed to set bypass mode\r\n");
            retVal = false;
        }

        uwb_pa = 0;
#else
        printf("Not supported\r\n");
        retVal = false;
#endif
        break;
    }
    case POWER_MODE_LOW: {
#if RF_BYPASS
        err = gpio_pin_set_dt(&_fem_gpios.bypass, 0);

        if (err) {
            printk("Failed to set bypass mode\r\n");
            retVal = false;
        }
#endif
#if HIGHLOW_POWER
        err = gpio_pin_set_dt(&_fem_gpios.power, 0);

        if (err) {
            printk("Failed to set bypass mode\r\n");
            retVal = false;
        }

        uwb_pa = 1;
#else
        printf("Not implemented\r\n");
        retVal = false;
#endif
        break;
    }
    case POWER_MODE_HIGH: {
#if RF_BYPASS
        err = gpio_pin_set_dt(&_fem_gpios.bypass, 0);

        if (err) {
            printk("Failed to set bypass mode\r\n");
            retVal = false;
            break;
        }
#endif
#if HIGHLOW_POWER
        err = gpio_pin_set_dt(&_fem_gpios.power, 1);

        if (err) {
            printk("Failed to set bypass mode\r\n");
            retVal = false;
            break;
        }

        uwb_pa = 1;
#else
        printf("Not implemented\r\n");
        retVal = false;
#endif
        break;
    }
    default:
        printf("Power mode not recognized\r\n");
        return false;
    }

    restore_bluetooth(ble_state);

    if (retVal) {
        dwt_setlnapamode(0, uwb_pa);
    }

    return true;
}

bool select_antenna(int32_t ant) {
#if ANTENNA_SELECT
    bool ble_state = save_and_disable_bluetooth();
    bool retVal = true;
    int err = 0;

    switch (ant) {
    case 1:
        err = gpio_pin_set_dt(&_fem_gpios.ant_sel, 0);
        break;
    case 2:
        err = gpio_pin_set_dt(&_fem_gpios.ant_sel, 1);
        break;
    default:
        printf("Invalid antenna selection\r\n");
        retVal = false;
    }

    restore_bluetooth(ble_state);

    if (err) {
        printk("Could not change antenna (%d)\n", err);
        return false;
    }
    return retVal;
#else
    printf("Not implemented\r\n");
    return false;
#endif
}

bool update_fem_shutdown_state(bool shutdown) {
    int err;

    err = gpio_pin_set_dt(&_fem_gpios.shutdown, shutdown);

    if (err) {
        printk("Could not shut down FEM (%d)", err);
        return false;
    }

    return true;
}

#else
bool init_range_extension(void) {
    LOG_INF("Range extension disabled");
    return true;
}

bool update_power_mode(enum ble_power_mode mode) {
    if (true) {
        printf("Not implemented\r\n");
    }
    return false;
}

bool select_antenna(int32_t ant) {
    printf("Not implemented\r\n");
    return false;
}

bool update_fem_shutdown_state(bool shutdown) {
    return false;
}
#endif
