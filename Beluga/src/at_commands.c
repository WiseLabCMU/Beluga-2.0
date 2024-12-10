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
#include <initiator.h>
#include <responder.h>

#include <at_commands.h>
#include <ble_app.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <uart.h>

#include <app_leds.h>
#include <list_monitor.h>
#include <list_neighbors.h>
#include <power_manager.h>
#include <range_extension.h>
#include <ranging.h>
#include <settings.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread_priorities.h>
#include <utils.h>
#include <watchdog.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(at_commands, LOG_LEVEL_DBG);

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

#define AT_CMD_DEFINE(_command)                                                \
    static void at_##_command(uint16_t argc, char const *const *argv)

#define CMD_DATA(_callback, _command, _command_len)                            \
    {                                                                          \
        .command = (const char *)(_command), .cmd_length = (_command_len),     \
        .cmd_func = (_callback),                                               \
    }

#define AT_CMD_DATA(_command...)                                               \
    CMD_DATA((at_##_command), ((uint8_t[]){#_command}),                        \
             (sizeof((uint8_t[]){#_command}) - 1))

#define AT_CMD_DATA_TERMINATOR                                                 \
    { NULL, 0, NULL }

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
    LOG_INF("Parsed %" PRIu16 " arguments", argc);

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

AT_CMD_DEFINE(STARTUWB) {
    LOG_INF("Running STARTUWB command");
    if (get_ble_led_state() == LED_BLE_OFF) {
        // Avoid undefined behavior
        printf("Cannot start UWB: BLE has not been started\r\n");
        return;
    }
    if (get_uwb_led_state() == LED_UWB_ON) {
        printf("UWB is already on\r\n");
        return;
    }
    if (retrieveSetting(BELUGA_RANGE_EXTEND) == 1) {
        update_power_mode(POWER_MODE_HIGH);
    } else {
        update_power_mode(POWER_MODE_BYPASS);
    }
    k_sem_give(&k_sus_resp);
    k_sem_give(&k_sus_init);
    update_led_state(LED_UWB_ON);
    OK;
}

AT_CMD_DEFINE(STOPUWB) {
    LOG_INF("Running STOPUWB command");
    if (get_uwb_led_state() == LED_UWB_OFF) {
        printf("UWB is not running\r\n");
        return;
    }
    k_sem_take(&k_sus_resp, K_FOREVER);
    k_sem_take(&k_sus_init, K_FOREVER);
    update_led_state(LED_UWB_OFF);
    OK;
}

AT_CMD_DEFINE(STARTBLE) {
    LOG_INF("Running STARTBLE) command");
    if (get_NODE_UUID() == 0) {
        printf("Cannot start BLE: Node ID is not set\r\n");
        return;
    } else if (get_ble_led_state() == LED_UWB_ON) {
        printf("BLE is already on\r\n");
        return;
    }
    k_sem_give(&print_list_sem);
    int err = enable_bluetooth();
    if (err) {
        printf("Failed to start BLE (%d)\r\n", err);
        k_sem_take(&print_list_sem, K_FOREVER);
        return;
    }
    update_led_state(LED_BLE_ON);
    OK;
}

AT_CMD_DEFINE(STOPBLE) {
    LOG_INF("Running STOPBLE command");
    if (get_ble_led_state() == LED_UWB_OFF) {
        printf("BLE is already off\r\n");
        return;
    }
    int err = disable_bluetooth();
    if (err) {
        printf("Failed to stop BLE (%d)\r\n", err);
        return;
    }
    k_sem_take(&print_list_sem, K_FOREVER);
    update_led_state(LED_BLE_OFF);
    OK;
}

AT_CMD_DEFINE(ID) {
    LOG_INF("Running ID command");
    READ_SETTING(argc, 2, BELUGA_ID, "ID");
    int32_t newID;
    bool success = strtoint32(argv[1], &newID);

    if (!success || newID <= 0 || newID > (int32_t)UINT16_MAX) {
        printf("Invalid ID\r\n");
        return;
    }

    if (set_initiator_id((uint16_t)newID) != 0) {
        printf("Unable to set ID: UWB currently active\r\n");
    }

    // We know that UWB is inactive at this point
    set_responder_id((uint16_t)newID);
    update_node_id((uint16_t)newID);
    updateSetting(BELUGA_ID, newID);
    OK;
}

AT_CMD_DEFINE(BOOTMODE) {
    LOG_INF("Running BOOTMODE command");
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

AT_CMD_DEFINE(RATE) {
    LOG_INF("Running RATE command");
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

AT_CMD_DEFINE(CHANNEL) {
    LOG_INF("Running CHANNEL command");
    READ_SETTING(argc, 2, BELUGA_UWB_CHANNEL, "Channel");
    int32_t channel;
    int retVal;
    bool success = strtoint32(argv[1], &channel);

    if (!success) {
        printf("Channel parameter input error \r\n");
        return;
    }

    retVal = set_uwb_channel(channel);
    if (retVal == -EBUSY) {
        printf("Cannot set UWB parameter: UWB is active \r\n");
        return;
    } else if (retVal != 0) {
        printf("Channel parameter input error \r\n");
        return;
    }

    updateSetting(BELUGA_UWB_CHANNEL, channel);
    OK;
}

AT_CMD_DEFINE(RESET) {
    LOG_INF("Running RESET command");
    resetBelugaSettings();
    printf("Reset ");
    OK;
}

AT_CMD_DEFINE(TIMEOUT) {
    LOG_INF("Running TIMEOUT command");
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

AT_CMD_DEFINE(TXPOWER) {
    LOG_INF("Running TXPOWER command");
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

AT_CMD_DEFINE(STREAMMODE) {
    LOG_INF("Running STREAMMODE command");
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

AT_CMD_DEFINE(TWRMODE) {
    LOG_INF("Running TWRMODE command");
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

AT_CMD_DEFINE(LEDMODE) {
    LOG_INF("Running LEDMODE command");
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

AT_CMD_DEFINE(REBOOT) {
    LOG_INF("Running REBOOT command");
    OK;
    printf("\r\n");
    disable_bluetooth();
    sys_reboot(SYS_REBOOT_COLD);
}

AT_CMD_DEFINE(PWRAMP) {
    LOG_INF("Running PWRAMP command");
    READ_SETTING(argc, 2, BELUGA_RANGE_EXTEND, "Range Extension");
    int32_t pwramp;
    bool success = strtoint32(argv[1], &pwramp);

    if (!success || pwramp < 0 || pwramp > 1) {
        printf("Power amp parameter input error \r\n");
        return;
    }

    if (pwramp == 0) {
        success = update_power_mode(POWER_MODE_BYPASS);
    } else {
        success = update_power_mode(POWER_MODE_HIGH);
    }

    if (success) {
        updateSetting(BELUGA_RANGE_EXTEND, pwramp);
        OK;
    }
}

AT_CMD_DEFINE(ANTENNA) {
    LOG_INF("Running ANTENNA command");
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

AT_CMD_DEFINE(TIME) {
    LOG_INF("Running TIME command");
    printf("Time: %" PRId64 " ", k_uptime_get());
    OK;
}

AT_CMD_DEFINE(FORMAT) {
    LOG_INF("Running FORMAT command");
    READ_SETTING(argc, 2, BELUGA_OUT_FORMAT, "Format Mode");
    int32_t mode;
    bool success = strtoint32(argv[1], &mode);

    if (!success || mode < 0 || mode > 1) {
        printf("Format parameter input error \r\n");
        return;
    }

    updateSetting(BELUGA_OUT_FORMAT, mode);
    set_format_mode(mode == 1);
    OK;
}

AT_CMD_DEFINE(DEEPSLEEP) {
    LOG_INF("Running DEEPSLEEP command");
    OK;
    printf("\r\n");
    enter_deep_sleep();
}

AT_CMD_DEFINE(PHR) {
    LOG_INF("Running PHR command");
    READ_SETTING(argc, 2, BELUGA_UWB_PHR, "UWB PHR mode");
    int32_t phr;
    int retVal;
    bool success = strtoint32(argv[1], &phr);

    if (!success) {
        printf("PHR mode parameter input error \r\n");
        return;
    }

    retVal = uwb_set_phr_mode((enum uwb_phr_mode)phr);
    if (retVal == -EBUSY) {
        printf("Cannot set UWB parameter: UWB is active \r\n");
        return;
    } else if (retVal != 0) {
        printf("PHR mode parameter input error \r\n");
        return;
    }

    updateSetting(BELUGA_UWB_PHR, phr);
    OK;
}

AT_CMD_DEFINE(DATARATE) {
    LOG_INF("Running DATARATE command");
    READ_SETTING(argc, 2, BELUGA_UWB_DATA_RATE, "UWB data rate");
    int32_t rate;
    int retVal;
    bool success = strtoint32(argv[1], &rate);

    if (!success) {
        printf("Data rate parameter input error \r\n");
        return;
    }

    retVal = uwb_set_datarate((enum uwb_datarate)rate);
    if (retVal == -EBUSY) {
        printf("Cannot set UWB parameter: UWB is active \r\n");
        return;
    } else if (retVal != 0) {
        printf("Data rate parameter input error \r\n");
        return;
    }

    updateSetting(BELUGA_UWB_DATA_RATE, rate);
    OK;
}

AT_CMD_DEFINE(PULSERATE) {
    LOG_INF("Running PULSERATE command");
    READ_SETTING(argc, 2, BELUGA_UWB_PULSE_RATE, "Pulse Rate");
    int32_t rate;
    int retVal;
    bool success = strtoint32(argv[1], &rate);

    if (!success) {
        printf("Invalid pulse rate input parameter \r\n");
        return;
    }

    retVal = uwb_set_pulse_rate((enum uwb_pulse_rate)rate);
    if (retVal == -EBUSY) {
        printf("Cannot set UWB parameter: UWB is active \r\n");
        return;
    } else if (retVal != 0) {
        printf("Pulse rate parameter input error \r\n");
        return;
    }

    updateSetting(BELUGA_UWB_PULSE_RATE, rate);
    OK;
}

AT_CMD_DEFINE(PREAMBLE) {
    LOG_INF("Running PREAMBLE command");
    READ_SETTING(argc, 2, BELUGA_UWB_PREAMBLE, "Preamble length");
    int32_t preamble;
    int retVal;
    bool success = strtoint32(argv[1], &preamble);

    if (!success) {
        printf("Invalid Preamble length setting \r\n");
        return;
    }

    retVal = uwb_set_preamble((enum uwb_preamble_length)preamble);
    if (retVal == -EBUSY) {
        printf("Cannot set UWB parameter: UWB is active \r\n");
        return;
    } else if (retVal != 0) {
        printf("Preamble parameter input error \r\n");
        return;
    }

    updateSetting(BELUGA_UWB_PREAMBLE, preamble);
    OK;
}

AT_CMD_DEFINE(PAC) {
    LOG_INF("Running PAC command");
    READ_SETTING(argc, 2, BELUGA_UWB_PAC, "PAC Size");
    int32_t pac_size;
    int retVal;
    bool success = strtoint32(argv[1], &pac_size);

    if (!success) {
        printf("Invalid PAC size setting\r\n");
        return;
    }

    retVal = set_pac_size((enum uwb_pac)pac_size);
    if (retVal == -EBUSY) {
        printf("Cannot set UWB parameter: UWB is active \r\n");
        return;
    } else if (retVal != 0) {
        printf("PAC Size parameter input error \r\n");
        return;
    }

    updateSetting(BELUGA_UWB_PAC, pac_size);
    OK;
}

AT_CMD_DEFINE(SFD) {
    LOG_INF("Running SFD command");
    READ_SETTING(argc, 2, BELUGA_UWB_NSSFD, "Nonstandard SFD");
    int32_t sfd;
    int retVal;
    bool success = strtoint32(argv[1], &sfd);

    if (!success) {
        printf("Invalid Preamble length setting \r\n");
        return;
    }

    retVal = set_sfd_mode((enum uwb_sfd)sfd);
    if (retVal == -EBUSY) {
        printf("Cannot set UWB parameter: UWB is active \r\n");
        return;
    } else if (retVal != 0) {
        printf("SFD parameter input error \r\n");
        return;
    }

    updateSetting(BELUGA_UWB_NSSFD, sfd);
    OK;
}

AT_CMD_DEFINE(PANID) {
    LOG_INF("Running PANID command");
    READ_SETTING(argc, 2, BELUGA_PAN_ID, "PAN ID");
    int32_t pan_id;
    int retVal;
    bool success = strtoint32(argv[1], &pan_id);

    if (!success || pan_id < INT32_C(0) || pan_id > (uint32_t)UINT16_MAX) {
        printf("Invalid PAN ID\r\n");
        return;
    }

    retVal = set_initiator_pan_id((uint16_t)pan_id);

    if (retVal != 0) {
        printf("Cannot set PAN ID: UWB Active\r\n");
        return;
    }
    set_responder_pan_id((uint16_t)pan_id);
    updateSetting(BELUGA_PAN_ID, pan_id);
    OK;
}

static struct cmd_info commands[] = {
    AT_CMD_DATA(STARTUWB), AT_CMD_DATA(STOPUWB),   AT_CMD_DATA(STARTBLE),
    AT_CMD_DATA(STOPBLE),  AT_CMD_DATA(ID),        AT_CMD_DATA(BOOTMODE),
    AT_CMD_DATA(RATE),     AT_CMD_DATA(CHANNEL),   AT_CMD_DATA(RESET),
    AT_CMD_DATA(TIMEOUT),  AT_CMD_DATA(TXPOWER),   AT_CMD_DATA(STREAMMODE),
    AT_CMD_DATA(TWRMODE),  AT_CMD_DATA(LEDMODE),   AT_CMD_DATA(REBOOT),
    AT_CMD_DATA(PWRAMP),   AT_CMD_DATA(ANTENNA),   AT_CMD_DATA(TIME),
    AT_CMD_DATA(FORMAT),   AT_CMD_DATA(DEEPSLEEP), AT_CMD_DATA(PHR),
    AT_CMD_DATA(DATARATE), AT_CMD_DATA(PULSERATE), AT_CMD_DATA(PREAMBLE),
    AT_CMD_DATA(PAC),      AT_CMD_DATA(SFD),       AT_CMD_DATA(PANID),
    AT_CMD_DATA_TERMINATOR};

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

    LOG_DBG("Starting commands task\n");

    struct buffer *commandBuffer = NULL;
    struct task_wdt_attr watchdogAttr = {.period = 2000};

    if (spawn_task_watchdog(&watchdogAttr) < 0) {
        printk("Unable to spawn task watchdog in command thread\n");
        while (1)
            ;
    }

    while (true) {
        k_msleep(100);

        commandBuffer = k_fifo_get(&uart_rx_queue, K_NO_WAIT);
        watchdog_red_rocket(&watchdogAttr);

        if (commandBuffer == NULL) {
            continue;
        }
        LOG_INF("Item received: %s\n", commandBuffer->buf);

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
                found = true;
                if (commands[i].cmd_func == NULL) {
                    printf("Not implemented\r\n");
                } else {
                    LOG_DBG("Command found");
                    argc = argparse(commandBuffer->buf, argv);
                    argv[argc] = NULL;
                    commands[i].cmd_func(argc, (const char **)argv);
                }
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
    k_thread_name_set(command_task_id, "Command thread");
    LOG_INF("Started AT commands");
}
#else
void init_commands_thread(void) { LOG_INF("Started AT commands"); }
#endif
