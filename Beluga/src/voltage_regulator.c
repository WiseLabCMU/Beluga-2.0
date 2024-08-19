//
// Created by tom on 8/13/24.
//

#include <voltage_regulator.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

static enum voltage_level currentLevel = VR_3V3;

#define VR_NODE_EXISTS DT_NODE_EXISTS(DT_NODELABEL(vr_adjust))

#if !VR_NODE_EXISTS
#define VR_RET_SUCCESS                                                         \
    do {                                                                       \
        printk("No voltage regulator node present\n");                         \
        return true;                                                           \
    } while (0)
#define VR_RET_LEVEL                                                           \
    do {                                                                       \
        printk(                                                                \
            "No voltage regulator node present. Returning default level\n");   \
        return VR_3V3;                                                         \
    } while (0)

static const struct gpio_dt_spec _vr_adjust;
#else
#define VR_RET_SUCCESS                                                         \
    do {                                                                       \
    } while (0)
#define VR_RET_LEVEL                                                           \
    do {                                                                       \
    } while (0)

static const struct gpio_dt_spec _vr_adjust =
    GPIO_DT_SPEC_GET(DT_NODELABEL(vr_adjust), gpios);
#endif

bool init_voltage_regulator(void) {
    VR_RET_SUCCESS;

    int err;
    if (!device_is_ready(_vr_adjust.port)) {
        printk("Voltage regulator gpio not ready\n");
        return false;
    }

    err = gpio_pin_configure_dt(&_vr_adjust, GPIO_DISCONNECTED);
    currentLevel = VR_3V3;
    if (!err) {
        printk("Voltage regulator configured\n");
    } else {
        printk("An error occurred when configuring voltage regulator\n");
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
        printk("Received an invalid voltage level\n");
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
