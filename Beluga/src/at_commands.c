/** @file   at_commands.c
 *
 *  @brief  Beluga AT commands
 *
 *  @date   2024/06
 *
 *  @author WiseLab-CMU
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

#include "deca_types.h"
#include "init_main.h"
#include "resp_main.h"

#include <at_commands.h>
#include <ble_app.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <uart.h>

#include <app_leds.h>
#include <list_monitor.h>
#include <list_neighbors.h>
#include <range_extension.h>
#include <ranging.h>
#include <settings.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread_priorities.h>
#include <utils.h>

#define OK         printf("OK\r\n")
#define MAX_TOKENS 20

#define CHECK_ARGC(argc, required)                                             \
    do {                                                                       \
        if ((uint16)(argc) < (uint16_t)(required)) {                           \
            printf("Missing argument(s)\r\n");                                 \
            return;                                                            \
        }                                                                      \
    } while (0)

#define READ_SETTING(argc, required, setting, settingstr)                      \
    do {                                                                       \
        if ((uint16_t)(argc) < (uint16_t)(required)) {                         \
            int32_t _setting = retrieveSetting(setting);                       \
            printf(settingstr ": %d ", _setting);                              \
            OK;                                                                \
            return;                                                            \
        }                                                                      \
    } while (0)

struct cmd_info {
    const char *command;
    size_t cmd_length;
    void (*cmd_func)(uint16_t argc, char const *const *argv);
};

STATIC_INLINE bool int2bool(bool *boolarg, int32_t intarg) {
    if (intarg == 0) {
        *boolarg = false;
    } else if (intarg == 1) {
        *boolarg = true;
    } else {
        return false;
    }
    return true;
}

static uint16_t argparse(char *s, char **argv) {
    char *temp;
    uint16_t argc;

    for (argc = 0, temp = s; argc < (MAX_TOKENS - 1); argc++) {
        while (isspace((int)*temp)) {
            temp++;
        }

        if (*temp == '\0') {
            break;
        }

        argv[argc] = temp;

        while (isgraph((int)*temp)) {
            temp++;
        }

        if (isspace((int)*temp)) {
            *temp = '\0';
            temp++;
        }
    }

    return argc;
}

static bool strtoint32(const char *str, int32_t *result) {
    char *endptr;
    unsigned long ret;

    errno = 0;
    ret = strtol(str, &endptr, 10);

    if (errno == ERANGE || (int64_t)ret > (int64_t)INT32_MAX ||
        isgraph((int)*endptr)) {
        *result = 0;
        return false;
    }

    *result = (int32_t)ret;
    return true;
}

static void at_start_uwb(UNUSED uint16_t argc, UNUSED char const *const *argv) {
    if (get_ble_led_state() == LED_BLE_OFF) {
        // Avoid undefined behavior
        printf("Cannot start UWB: BLE has not been started\r\n");
        return;
    }
    k_sem_give(&k_sus_resp);
    k_sem_give(&k_sus_init);
    update_led_state(LED_UWB_ON);
    OK;
}

static void at_stop_uwb(UNUSED uint16_t argc, UNUSED char const *const *argv) {
    k_sem_take(&k_sus_resp, K_FOREVER);
    k_sem_take(&k_sus_init, K_FOREVER);
    update_led_state(LED_UWB_OFF);
    OK;
}

static void at_start_ble(UNUSED uint16_t argc, UNUSED char const *const *argv) {
    if (get_NODE_UUID() == 0) {
        printf("Cannot start BLE: Node ID is not set\r\n");
        return;
    }
    k_sem_give(&print_list_sem);
    enable_bluetooth();
    update_led_state(LED_BLE_ON);
    OK;
}

static void at_stop_ble(UNUSED uint16_t argc, UNUSED char const *const *argv) {
    disable_bluetooth();
    k_sem_take(&print_list_sem, K_FOREVER);
    update_led_state(LED_BLE_OFF);
    OK;
}

static void at_change_id(uint16_t argc, char const *const *argv) {
    READ_SETTING(argc, 2, BELUGA_ID, "ID");
    int32_t newID;
    bool success = strtoint32(argv[1], &newID);

    if (!success || newID <= 0 || newID > (int32_t)UINT16_MAX) {
        printf("Invalid ID\r\n");
        return;
    }

    update_node_id((uint16_t)newID);
    updateSetting(BELUGA_ID, newID);
    OK;
}

static void at_bootmode(uint16_t argc, char const *const *argv) {
    READ_SETTING(argc, 2, BELUGA_BOOTMODE, "Bootmode");
    int32_t mode;
    bool success = strtoint32(argv[1], &mode);

    if (mode < 0 || mode > 2 || !success) {
        printf("Invalid bootmode parameter \r\n");
        return;
    }

    updateSetting(BELUGA_BOOTMODE, mode);
    printf("Bootmode: %d ", mode);
    OK;
}

static void at_rate(uint16_t argc, char const *const *argv) {
    READ_SETTING(argc, 2, BELUGA_POLL_RATE, "Rate");
    int32_t rate;
    bool success = strtoint32(argv[1], &rate);

    if (rate < 0 || rate > 500 || !success) {
        printf("Invalid rate parameter\r\n");
        return;
    }

    updateSetting(BELUGA_POLL_RATE, rate);
    set_rate(rate);

    // reconfig ble data
    advertising_reconfig(rate != 0);
    printf("Rate: %d ", rate);
    OK;
}

static void at_channel(uint16_t argc, char const *const *argv) {
    READ_SETTING(argc, 2, BELUGA_UWB_CHANNEL, "Channel");
    int32_t channel;
    bool success = strtoint32(argv[1], &channel);

    if (success && set_uwb_channel(channel)) {
        updateSetting(BELUGA_UWB_CHANNEL, channel);
        OK;
    } else {
        printf("Invalid channel\r\n");
    }
}

static void at_reset(UNUSED uint16_t argc, UNUSED char const *const *argv) {
    resetBelugaSettings();
    printf("Reset ");
    OK;
}

static void at_timeout(uint16_t argc, char const *const *argv) {
    READ_SETTING(argc, 2, BELUGA_BLE_TIMEOUT, "Timeout");
    int32_t timeout;
    bool success = strtoint32(argv[1], &timeout);

    if (!success || timeout < 0) {
        printf("Invalid timeout value\r\n");
        return;
    }

    updateSetting(BELUGA_BLE_TIMEOUT, timeout);
    set_node_timeout(timeout);
    OK;
}

static void at_txpower(uint16_t argc, char const *const *argv) {
    READ_SETTING(argc, 2, BELUGA_TX_POWER, "TX Power");
    int32_t power;
    bool value, success = strtoint32(argv[1], &power);

    if (success && int2bool(&value, power)) {
        updateSetting(BELUGA_TX_POWER, value);
        set_tx_power(value);
        OK;
    } else {
        printf("Tx power parameter input error\r\n");
    }
}

static void at_streammode(uint16_t argc, char const *const *argv) {
    READ_SETTING(argc, 2, BELUGA_STREAMMODE, "Stream Mode");
    int32_t mode;
    bool value, success = strtoint32(argv[1], &mode);

    if (success && int2bool(&value, mode)) {
        updateSetting(BELUGA_STREAMMODE, mode);
        set_stream_mode(value);
        OK;
    } else {
        printf("Stream mode parameter input error \r\n");
    }
}

static void at_twrmode(uint16_t argc, char const *const *argv) {
    READ_SETTING(argc, 2, BELUGA_TWR, "TWR");
    int32_t twr;
    bool value, success = strtoint32(argv[1], &twr);

    if (success && int2bool(&value, twr)) {
        updateSetting(BELUGA_TWR, twr);
        set_twr_mode(value);
        OK;
    } else {
        printf("TWR mode parameter input error \r\n");
    }
}

static void at_ledmode(uint16_t argc, char const *const *argv) {
    READ_SETTING(argc, 2, BELUGA_LEDMODE, "LED Mode");
    int32_t mode;
    bool success = strtoint32(argv[1], &mode);

    if (!success || mode < 0 || mode > 1) {
        printf("LED mode parameter input error \r\n");
        return;
    }

    updateSetting(BELUGA_LEDMODE, mode);
    if (mode == 1) {
        all_leds_off();
    } else {
        restore_led_states();
    }

    OK;
}

static void at_reboot(UNUSED uint16_t argc, UNUSED char const *const *argv) {
    OK;
    printf("\r\n");
    disable_bluetooth();
    sys_reboot(SYS_REBOOT_COLD);
}

static void at_pwramp(uint16_t argc, char const *const *argv) {
    CHECK_ARGC(argc, 2);
    int32_t pwramp;
    bool success = strtoint32(argv[1], &pwramp);

    if (!success || pwramp < 0 || pwramp > 1) {
        printf("Power amp parameter input error \r\n");
        return;
    }

    if (pwramp == 0) {
        success = disable_range_extension();
    } else {
        success = enable_range_extension();
    }

    if (success) {
        OK;
    }
}

static void at_antenna(uint16_t argc, char const *const *argv) {
    CHECK_ARGC(argc, 2);
    int32_t antenna;
    bool success = strtoint32(argv[1], &antenna);

    if (!success || antenna < 1 || antenna > 2) {
        printf("Antenna parameter input error \r\n");
        return;
    }

    success = select_antenna(antenna);

    if (success) {
        OK;
    }
}

static void at_time(uint16_t argc, char const *const *argv) {
    printf("Time: %lld ", k_uptime_get());
    OK;
}

static struct cmd_info commands[] = {{"STARTUWB", 8, at_start_uwb},
                                     {"STOPUWB", 7, at_stop_uwb},
                                     {"STARTBLE", 8, at_start_ble},
                                     {"STOPBLE", 7, at_stop_ble},
                                     {"ID", 2, at_change_id},
                                     {"BOOTMODE", 8, at_bootmode},
                                     {"RATE", 4, at_rate},
                                     {"CHANNEL", 7, at_channel},
                                     {"RESET", 5, at_reset},
                                     {"TIMEOUT", 7, at_timeout},
                                     {"TXPOWER", 7, at_txpower},
                                     {"STREAMMODE", 10, at_streammode},
                                     {"TWRMODE", 7, at_twrmode},
                                     {"LEDMODE", 7, at_ledmode},
                                     {"REBOOT", 6, at_reboot},
                                     {"PWRAMP", 6, at_pwramp},
                                     {"ANTENNA", 7, at_antenna},
                                     {"TIME", 4, at_time},
                                     {NULL, 0, NULL}};

STATIC_INLINE void freeCommand(struct buffer **buf) {
    k_free((*buf)->buf);
    k_free(*buf);
    *buf = NULL;
}

/**
 * @brief Task to receive UART message from zephyr UART queue and parse
 */
NO_RETURN void runSerialCommand(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    bool found = false;
    char *argv[MAX_TOKENS];
    uint16_t argc;

    struct buffer *commandBuffer = NULL;

    while (true) {
        k_msleep(100);

        commandBuffer = k_fifo_get(&uart_rx_queue, K_NO_WAIT);

        if (commandBuffer == NULL) {
            continue;
        }
        printk("Item received\n");

        if (0 != strncmp((const char *)commandBuffer->buf, "AT+", 3)) {
            if (0 == strncmp((const char *)commandBuffer->buf, "AT", 2)) {
                printf("Only input AT without + command \r\n");
            } else {
                printf("Not an AT command\r\n");
            }
            freeCommand(&commandBuffer);
            continue;
        }

        if (commandBuffer->len == 3) {
            printf("No command found after AT+\r\n");
            freeCommand(&commandBuffer);
            continue;
        }

        for (size_t i = 0; commands[i].command != NULL; i++) {
            if (0 == strncmp((const char *)(commandBuffer->buf + 3),
                             commands[i].command, commands[i].cmd_length)) {
                argc = argparse(commandBuffer->buf, argv);
                argv[argc] = NULL;

                if (commands[i].cmd_func != NULL) {
                    commands[i].cmd_func(argc, (const char **)argv);
                } else {
                    printf("Not implemented\r\n");
                }

                found = true;
                break;
            }
        }

        if (!found) {
            printf("ERROR Invalid AT Command\r\n");
        }
        found = false;
        freeCommand(&commandBuffer);
    }
}

#if ENABLE_THREADS && ENABLE_COMMANDS
K_THREAD_STACK_DEFINE(command_stack, CONFIG_COMMANDS_STACK_SIZE);
static struct k_thread command_thread_data;
static k_tid_t command_task_id;

void init_commands_thread(void) {
    command_task_id = k_thread_create(
        &command_thread_data, command_stack,
        K_THREAD_STACK_SIZEOF(command_stack), runSerialCommand, NULL, NULL,
        NULL, CONFIG_BELUGA_COMMANDS_PRIO, 0, K_NO_WAIT);
    printk("Started AT commands\n");
}
#else
void init_commands_thread(void) { printk("Started AT commands\n"); }
#endif

// int leds_mode;
// static uwb_lna_status = 0;
// static uwb_pa_status = 0;
