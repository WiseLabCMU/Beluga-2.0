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
#endif

#if !DT_NODE_HAS_PROP(SKY_GPIOS, csd_gpios)
#error "Shutdown gpio property (csd-gpios) is missing"
#endif

#if DT_NODE_HAS_PROP(SKY_GPIOS, cps_gpios)
#define RF_BYPASS 1
#endif

#if DT_NODE_HAS_PROP(SKY_GPIOS, chl_gpios)
#define HIGHLOW_POWER 1
#endif

#define _GEN_STRUCT_MEMBER(member) const struct gpio_dt_spec member

#define GEN_STRUCT_MEMBER(enabled, member)                                     \
    COND_CODE_1(IS_ENABLED(enabled), (_GEN_STRUCT_MEMBER(member)), ())

#define _INIT_STRUCT_MEMBER(member, nodelabel, prop)                           \
    .member = GPIO_DT_SPEC_GET(nodelabel, prop)

#define INIT_STRUCT_MEMBER(enabled, member, nodelabel, prop)                   \
    COND_CODE_1(IS_ENABLED(enabled),                                           \
                (_INIT_STRUCT_MEMBER(member, nodelabel, prop)), ())

static struct fem_gpios {
    GEN_STRUCT_MEMBER(ANTENNA_SELECT, ant_sel);
    GEN_STRUCT_MEMBER(1, shutdown);
    GEN_STRUCT_MEMBER(RF_BYPASS, bypass);
    GEN_STRUCT_MEMBER(HIGHLOW_POWER, power);
} _fem_gpios = {
    INIT_STRUCT_MEMBER(ANTENNA_SELECT, ant_sel, SKY_GPIOS, ant_sel_gpios),
    INIT_STRUCT_MEMBER(1, shutdown, SKY_GPIOS, csd_gpios),
    INIT_STRUCT_MEMBER(RF_BYPASS, bypass, SKY_GPIOS, cps_gpios),
    INIT_STRUCT_MEMBER(HIGHLOW_POWER, power, SKY_GPIOS, chl_gpios),
};

#define _INIT_FEM_PIN(container, attr, config)                                 \
    do {                                                                       \
        int err;                                                               \
        if (!device_is_ready((container).attr.port)) {                         \
            LOG_ERR(#attr " was not ready\n");                                 \
            return -ENODEV;                                                    \
        }                                                                      \
        err = gpio_pin_configure_dt(&(container).attr, config);                \
        if (err) {                                                             \
            LOG_ERR(#attr " configure (%d)\n", err);                           \
            return err;                                                        \
        }                                                                      \
    } while (0)

#define INIT_FEM_PIN(enabled, container, attr, config)                         \
    COND_CODE_1(IS_ENABLED(enabled), (_INIT_FEM_PIN(container, attr, config)), \
                ())

int init_range_extension(void) {
    INIT_FEM_PIN(ANTENNA_SELECT, _fem_gpios, ant_sel, GPIO_OUTPUT_INACTIVE);
    INIT_FEM_PIN(1, _fem_gpios, shutdown, GPIO_OUTPUT_LOW);
    INIT_FEM_PIN(RF_BYPASS, _fem_gpios, bypass, GPIO_OUTPUT_HIGH);
    INIT_FEM_PIN(HIGHLOW_POWER, _fem_gpios, power, GPIO_OUTPUT_LOW);
    return 0;
}

#define GEN_EXTRA_ACTION(...)                                                  \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (), (GET_ARG_N(1, __VA_ARGS__)))

#define _UPDATE_FEM_PIN(container, attr, value, ret, ...)                      \
    do {                                                                       \
        int err = gpio_pin_set_dt(&(container).attr, (value));                 \
        if (err != 0) {                                                        \
            LOG_ERR("Failed to update " #attr " to " #value " (err %d)", err); \
            (ret) = err;                                                       \
        }                                                                      \
        GEN_EXTRA_ACTION(__VA_ARGS__);                                         \
    } while (0)

#define FEM_PIN_NOTSUP(ret)                                                    \
    do {                                                                       \
        printf("Not supported\r\n");                                           \
        (ret) = -ENOTSUP;                                                      \
    } while (0)

#define UPDATE_FEM_PIN(enabled, container, attr, value, ret, ...)              \
    COND_CODE_1(IS_ENABLED(enabled),                                           \
                (_UPDATE_FEM_PIN(container, attr, value, ret, __VA_ARGS__)),   \
                (FEM_PIN_NOTSUP(ret)))

int update_power_mode(enum ble_power_mode mode) {
    int ret = 0;
    int uwb_pa = 0;
    bool ble_state = save_and_disable_bluetooth();

    switch (mode) {
    case POWER_MODE_BYPASS: {
        UPDATE_FEM_PIN(RF_BYPASS, _fem_gpios, bypass, 1, ret, uwb_pa = 0);
        break;
    }
    case POWER_MODE_LOW: {
        UPDATE_FEM_PIN(RF_BYPASS, _fem_gpios, bypass, 0, ret);
        UPDATE_FEM_PIN(HIGHLOW_POWER, _fem_gpios, power, 0, ret, uwb_pa = 1);
        break;
    }
    case POWER_MODE_HIGH: {
        UPDATE_FEM_PIN(RF_BYPASS, _fem_gpios, bypass, 0, ret);
        UPDATE_FEM_PIN(HIGHLOW_POWER, _fem_gpios, power, 1, ret, uwb_pa = 1);
        break;
    }
    default:
        printf("Power mode not recognized\r\n");
        ret = -EINVAL;
        break;
    }

    restore_bluetooth(ble_state);

    if (ret == 0) {
        dwt_setlnapamode(0, uwb_pa);
    }

    return ret;
}

int select_antenna(int32_t ant) {
#if defined(ANTENNA_SELECT)
    bool ble_state = save_and_disable_bluetooth();
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
        err = -EINVAL;
        break;
    }

    restore_bluetooth(ble_state);

    if (err == 0) {
        LOG_ERR("Could not change antenna (%d)", err);
    }
    return err;
#else
    printf("Not implemented\r\n");
    return -ENOTSUP;
#endif
}

int update_fem_shutdown_state(bool shutdown) {
    int err = gpio_pin_set_dt(&_fem_gpios.shutdown, shutdown);

    if (err != 0) {
        LOG_ERR("Could not shut down FEM (%d)", err);
    }

    return err;
}

#else
int init_range_extension(void) {
    LOG_INF("Range extension disabled");
    return 0;
}

int update_power_mode(enum ble_power_mode mode) {
    if (mode == POWER_MODE_BYPASS) {
        return 0;
    }
    printf("Not implemented\r\n");
    return -ENOTSUP;
}

int select_antenna(int32_t ant) {
    ARG_UNUSED(ant);
    printf("Not implemented\r\n");
    return -ENOTSUP;
}

int update_fem_shutdown_state(bool shutdown) {
    ARG_UNUSED(shutdown);
    return 0;
}
#endif
