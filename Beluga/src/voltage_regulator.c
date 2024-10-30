//
// Created by tom on 8/13/24.
//

#include <voltage_regulator.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(vr_logger, CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL);

static enum voltage_level currentLevel = VR_3V3;

#define VR_NODE_EXISTS DT_NODE_EXISTS(DT_NODELABEL(vr_adjust))

#if !VR_NODE_EXISTS
#define VR_RET_SUCCESS                                                         \
    do {                                                                       \
        LOG_WRN("No voltage regulator node present");                          \
        return true;                                                           \
    } while (0)
#define VR_RET_LEVEL                                                           \
    do {                                                                       \
        LOG_WRN("No voltage regulator node present. Returning default level"); \
        return VR_3V3;                                                         \
    } while (0)

static const struct gpio_dt_spec _vr_adjust;
#else
#define VR_RET_SUCCESS (void)0
#define VR_RET_LEVEL   (void)0

static const struct gpio_dt_spec _vr_adjust =
    GPIO_DT_SPEC_GET(DT_NODELABEL(vr_adjust), gpios);
#endif

bool init_voltage_regulator(void) {
    VR_RET_SUCCESS;

    int err;
    if (!device_is_ready(_vr_adjust.port)) {
        LOG_ERR("Voltage regulator gpio not ready");
        return false;
    }

    err = gpio_pin_configure_dt(&_vr_adjust, GPIO_DISCONNECTED);
    currentLevel = VR_3V3;
    if (!err) {
        LOG_INF("Voltage regulator configured");
    } else {
        LOG_ERR("An error occurred when configuring voltage regulator");
    }
    return err == 0;
}

bool update_voltage_level(enum voltage_level level) {
    VR_RET_SUCCESS;

    int err;

    switch (level) {
    case VR_2V4:
        err = gpio_pin_configure_dt(&_vr_adjust, GPIO_OUTPUT_ACTIVE);
        break;
    case VR_3V3:
        err = gpio_pin_configure_dt(&_vr_adjust, GPIO_DISCONNECTED);
        break;
    case VR_3V5:
        err = gpio_pin_configure_dt(&_vr_adjust, GPIO_OUTPUT_INACTIVE);
        break;
    default:
        LOG_WRN("Received an invalid voltage level");
        err = 1;
        break;
    }

    if (!err) {
        currentLevel = level;
    }

    return err == 0;
}

enum voltage_level get_current_voltage(void) {
    VR_RET_LEVEL;
    return currentLevel;
}
