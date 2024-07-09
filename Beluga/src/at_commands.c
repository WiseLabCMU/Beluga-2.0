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

#include <at_commands.h>
#include <ble_app.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <uart.h>

#include <flash.h>
#include <led_config.h>
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

struct cmd_info {
    const char *command;
    size_t cmd_length;
    void (*cmd_func)(uint16_t argc, char const *const *argv);
};

// TODO: File static vars?

static uint16_t argparse(char *s, char **argv) {
    char *temp;
    uint16_t argc;

    for (argc = 0, temp = s; argc < (MAX_TOKENS - 1); argc++) {
        while (isspace(*temp)) {
            temp++;
        }

        if (*temp == '\0') {
            break;
        }

        argv[argc] = temp;

        while (isgraph(*temp)) {
            temp++;
        }

        if (isspace(*temp)) {
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
        isgraph(*endptr)) {
        *result = 0;
        return false;
    }

    *result = (int32_t)ret;
    return true;
}

void init_at_commands(void) {
    // Init any semaphores here
    // Init UART queue here
}

static void at_start_uwb(UNUSED uint16_t argc, UNUSED char const *const *argv) {
    // Mark UWB as started
    // Post Semaphores

    /*if (leds_mode) {
        APP_LED_ON(UWB_LED);
    }*/
    OK;
}

static void at_stop_uwb(UNUSED uint16_t argc, UNUSED char const *const *argv) {
    // Mark UWB as stopped
    // Wait semaphores here

    //    if (leds_mode) {
    //        APP_LED_OFF(UWB_LED);
    //    }
    OK;
}

static void at_start_ble(UNUSED uint16_t argc, UNUSED char const *const *argv) {
    enable_bluetooth();
    // Post list semaphore

    //    if (leds_mode) {
    //        APP_LED_ON(BLE_LED);
    //    }
    OK;
}

static void at_stop_ble(UNUSED uint16_t argc, UNUSED char const *const *argv) {
    disable_bluetooth();
    // Wait list semaphore

    //    if (leds_mode) {
    //        APP_LED_OFF(BLE_LED);
    //    }

    OK;
}

static void at_change_id(uint16_t argc, char const *const *argv) {
    CHECK_ARGC(argc, 2);
    int32_t newID;
    bool success = strtoint32(argv[1], &newID);

    if (!success || newID <= 0 || newID > (int32_t)UINT16_MAX) {
        printf("Invalid ID\r\n");
        return;
    }

    update_node_id((uint16_t)newID);
    writeFlashID(CONFIG_ID, newID);
    OK;
}

static void at_bootmode(uint16_t argc, char const *const *argv) {
    CHECK_ARGC(argc, 2);
    int32_t mode;
    bool success = strtoint32(argv[1], &mode);

    if (mode < 0 || mode > 2 || !success) {
        printf("Invalid bootmode parameter \r\n");
        return;
    }

    writeFlashID(CONFIG_BM, mode);
    printf("Bootmode: %d ", mode);
    OK;
}

static void at_rate(uint16_t argc, char const *const *argv) {
    CHECK_ARGC(argc, 2);
    int32_t rate;
    bool success = strtoint32(argv[1], &rate);

    if (rate < 0 || rate > 500 || !success) {
        printf("Invalid rate parameter\r\n");
        return;
    }

    writeFlashID(CONFIG_RATE, rate);
    // TODO: Set rate

    // reconfig ble data
    advertising_reconfig(rate != 0);
    printf("Rate: %d ", rate);
    OK;
}

static void at_channel(uint16_t argc, char const *const *argv) {
    CHECK_ARGC(argc, 2);
    int32_t channel;
    bool success = strtoint32(argv[1], &channel);

    if (success /* && set_uwb_pgdelay(channel) */) {
        OK;
    } else {
        printf("Invalid channel\r\n");
    }
}

static void at_reset(UNUSED uint16_t argc, UNUSED char const *const *argv) {
    eraseRecords();
    printf("Reset ");
    OK;
}

static void at_timeout(uint16_t argc, char const *const *argv) {
    CHECK_ARGC(argc, 2);
    int32_t timeout;
    bool success = strtoint32(argv[1], &timeout);

    if (!success || timeout < 0) {
        printf("Invalid timeout value\r\n");
        return;
    }

    writeFlashID(CONFIG_TIME, timeout);
    // TODO: set the timeout value
    OK;
}

static void at_txpower(uint16_t argc, char const *const *argv) {
    CHECK_ARGC(argc, 2);
    int32_t power;
    bool success = strtoint32(argv[1], &power);

    if (success /* && set_uwb_tx_power(power)*/) {
        OK;
    } else {
        printf("Tx power parameter input error\r\n");
    }
}

static void at_streammode(uint16_t argc, char const *const *argv) {
    CHECK_ARGC(argc, 2);
    int32_t mode;
    bool success = strtoint32(argv[1], &mode);

    if (success /*&& set_streaming_mode(mode)*/) {
        OK;
    } else {
        printf("Stream mode parameter input error \r\n");
    }
}

static void at_twrmode(uint16_t argc, char const *const *argv) {
    CHECK_ARGC(argc, 2);
    int32_t twr;
    bool success = strtoint32(argv[1], &twr);

    if (success /*&& set_twr_mode(twr)*/) {
        OK;
    } else {
        printf("TWR mode parameter input error \r\n");
    }
}

static void at_ledmode(uint16_t argc, char const *const *argv) {
    CHECK_ARGC(argc, 2);
    int32_t mode;
    bool success = strtoint32(argv[1], &mode);

    if (!success || mode < 0 || mode > 1) {
        printf("LED mode parameter input error \r\n");
        return;
    }

    writeFlashID(CONFIG_LED, mode);
    // TODO: Set mode
    if (mode == 1) {
        // TODO: Turn off LEDs
        // dwt_setleds(DWT_LEDS_DISABLE);
    } else {
        uint32_t state;
        // TODO: Turn on power LED
        // dwt_setleds(DWT_LEDS_ENABLE);

        state = readFlashID(CONFIG_BM);
        if (state > 0) {
            APP_LED_ON(UWB_LED);
        }
        if (state > 1) {
            APP_LED_ON(BLE_LED);
        }
    }

    OK;
}

static void at_reboot(UNUSED uint16_t argc, UNUSED char const *const *argv) {
    OK;
    sys_reboot(SYS_REBOOT_COLD);
}

static void at_pwramp(UNUSED uint16_t argc, UNUSED char const *const *argv) {
    printf("Not Implemented \r\n");
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
                                     {NULL, 0, NULL}};

static inline void freeCommand(struct buffer **buf) {
    k_free((*buf)->buf);
    k_free(*buf);
    *buf = NULL;
}

/**
 * @brief Task to receive UART message from zephyr UART queue and parse
 */
void runSerialCommand(void) {
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

#if ENABLE_THREADS
K_THREAD_DEFINE(command_task_id, STACK_SIZE, runSerialCommand, NULL, NULL, NULL,
                COMMAND_PRIO, 0, 0);
#endif

// SemaphoreHandle_t sus_resp, sus_init, print_list_sem;
//
// static int uwb_started;
// extern int ble_started;
//
// int leds_mode;
// static uwb_lna_status = 0;
// static uwb_pa_status = 0;
