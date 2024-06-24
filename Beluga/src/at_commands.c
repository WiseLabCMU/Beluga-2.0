/** @file   at_commands.c
 *
 *  @brief  Beluga AT commands
 *
 *  @date   2024/06
 *
 *  @author WiseLab-CMU
 */

#include "deca_types.h"
//#include "init_main.h"

#include <at_commands.h>
#include <ble_app.h>
//#include "uart.h"
#include <ctype.h>
#include <errno.h>
#include <string.h>

#include <flash.h>
#include <utils.h>
#include <stdlib.h>
#include <led_config.h>
#include <stdio.h>

#define BUF_SIZE 128
#define OK printf("OK\r\n")

struct cmd_info {
    const char *command;
    size_t cmd_length;
    void (*cmd_func)(const char *);
};

// TODO: File static vars?

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

void at_start_uwb(UNUSED const char *cmd) {
    // Mark UWB as started
    // Post Semaphores

    if (leds_mode) {
        APP_LED_ON(UWB_LED);
    }
    OK;
}

void at_stop_uwb(UNUSED const char *cmd) {
    // Mark UWB as stopped
    // Wait semaphores here

    if (leds_mode) {
        APP_LED_OFF(UWB_LED);
    }
    OK;
}

void at_start_ble(UNUSED const char *cmd) {
    enable_bluetooth();
    // Post list semaphore

    if (leds_mode) {
        APP_LED_ON(BLE_LED);
    }
    OK;
}

void at_stop_ble(UNUSED const char *cmd) {
    disable_bluetooth();
    // Wait list semaphore

    if (leds_mode) {
        APP_LED_OFF(BLE_LED);
    }

    OK;
}

void at_change_id(UNUSED const char *cmd) {
    char buf[BUF_SIZE];
    memcpy(buf, cmd, MIN(BUF_SIZE - 1, strlen(cmd)));
    char *uuid = strtok(buf, " ");
    uuid = strtok(NULL, " ");
    int32_t newID;
    bool success = strtoint32(uuid, &newID);

    if (!success || newID <= 0 || newID > (int32_t)UINT16_MAX) {
        printf("Invalid ID\r\n");
        return;
    }

    update_node_id((uint16_t)newID);
    writeFlashID(CONFIG_ID, newID);
    OK;
}

//
//extern ble_uuid_t m_adv_uuids[2];
//
//QueueHandle_t uart_queue;
//SemaphoreHandle_t sus_resp, sus_init, print_list_sem;
//
//static int uwb_started;
//extern int ble_started;
//
//int leds_mode;
//static uwb_lna_status = 0;
//static uwb_pa_status = 0;
//
//static void at_command_bootmode(const char *message) {
//    char buf[100];
//    strcpy(buf, message);
//    char *uuid_char = strtok(buf, " ");
//    uuid_char = strtok(NULL, " ");
//    uint32_t mode;
//    bool success = strtouint32(uuid_char, &mode);
//
//    if (mode < 0 || mode > 2 || !success) {
//        printf("Invalid bootmode parameter \r\n");
//    } else {
//        if (mode == 1) {
//            writeFlashID(1, 2);
//        } else if (mode == 2) {
//            writeFlashID(2, 2);
//        } else {
//            writeFlashID(0, 2);
//        }
//
//        printf("Bootmode: %d OK \r\n", mode);
//    }
//}
//
//static void at_command_rate(const char *message) {
//    char buf[100];
//    strcpy(buf, message);
//    char *uuid_char = strtok(buf, " ");
//    uuid_char = strtok(NULL, " ");
//    uint32_t rate;
//    bool success = strtouint32(uuid_char, &rate);
//
//    if (rate < 0 || rate > 500 || !success) {
//        printf("Invalid rate parameter \r\n");
//    } else {
//        writeFlashID(rate, 3);
//        set_initiator_freq(rate);
//
//        // reconfig BLE advertising data
//        if (rate == 0) {
//            advertising_reconfig(0);
//        } else {
//            advertising_reconfig(1);
//        }
//
//        printf("Rate: %d OK \r\n", rate);
//    }
//}
//
//static void at_command_channel(const char *message) {
//    char buf[100];
//    strcpy(buf, message);
//    char *uuid_char = strtok(buf, " ");
//    uuid_char = strtok(NULL, " ");
//    uint32_t channel;
//    bool success = strtouint32(uuid_char, &channel);
//
//    if (success && set_uwb_pgdelay(channel)) {
//        printf("OK\r\n");
//    } else {
//        printf("Invalid Channel number \r\n");
//    }
//}
//
//static void _at_helper_delete_record(uint16_t fileID, uint16_t recordKey) {
//    fds_record_desc_t record_desc;
//    fds_find_token_t ftok;
//    memset(&ftok, 0x00, sizeof(fds_find_token_t));
//    if (fds_record_find(fileID, recordKey, &record_desc, &ftok) ==
//        FDS_SUCCESS) {
//        ret_code_t ret = fds_record_delete(&record_desc);
//        if (ret != FDS_SUCCESS) {
//            printf("FDS Delete error \r\n");
//        }
//    }
//}
//
//static void at_command_reset(UNUSED const char *message) {
//    // Delete ID record
//    _at_helper_delete_record(FILE_ID, RECORD_KEY_1);
//
//    // Delete rate record
//    _at_helper_delete_record(FILE_ID, RECORD_KEY_3);
//
//    // Delete channel record
//    _at_helper_delete_record(FILE_ID, RECORD_KEY_4);
//
//    // Delete timeout record
//    _at_helper_delete_record(FILE_ID, RECORD_KEY_5);
//
//    // Delete Tx power record
//    _at_helper_delete_record(FILE_ID, RECORD_KEY_6);
//
//    // Delete stream mode record
//    _at_helper_delete_record(FILE_ID, RECORD_KEY_7);
//
//    // Delete TWR mode record
//    _at_helper_delete_record(FILE_ID, RECORD_KEY_8);
//
//    // Delete LED mode record
//    _at_helper_delete_record(FILE_ID, RECORD_KEY_9);
//
//    // Delete BOOT mode record
//    _at_helper_delete_record(FILE_ID, RECORD_KEY_2);
//
//    printf("Reset OK \r\n");
//}
//
//static void at_command_timeout(const char *message) {
//    char buf[100];
//    strcpy(buf, message);
//    char *uuid_char = strtok(buf, " ");
//    uuid_char = strtok(NULL, " ");
//    uint32_t timeout;
//    bool success = strtouint32(uuid_char, &timeout);
//    // printf("%d \r\n", timeout);
//
//    if (!success) {
//        printf("Timeout cannot be negative \r\n");
//    } else {
//        writeFlashID(timeout, 5);
//        set_time_out(timeout);
//
//        printf("OK \r\n");
//    }
//}
//
//static void at_command_txpower(const char *message) {
//    char buf[100];
//    strcpy(buf, message);
//    char *uuid_char = strtok(buf, " ");
//    uuid_char = strtok(NULL, " ");
//    uint32_t tx_power;
//    bool success = strtouint32(uuid_char, &tx_power);
//    // printf("%d \r\n", timeout);
//
//    if (success && set_uwb_tx_power(tx_power)) {
//        printf("OK \r\n");
//    } else {
//        printf("Tx Power parameter input error \r\n");
//    }
//}
//
//static void at_command_streammode(const char *message) {
//    char buf[100];
//    strcpy(buf, message);
//    char *uuid_char = strtok(buf, " ");
//    uuid_char = strtok(NULL, " ");
//    uint32_t stream_mode;
//    bool success = strtouint32(uuid_char, &stream_mode);
//
//    if (success && set_streaming_mode(stream_mode)) {
//        printf("OK \r\n");
//    } else {
//        printf("Stream mode parameter input error \r\n");
//    }
//}
//
//static void at_command_twrmode(const char *message) {
//    char buf[100];
//    strcpy(buf, message);
//    char *uuid_char = strtok(buf, " ");
//    uuid_char = strtok(NULL, " ");
//    uint32_t ranging_mode;
//    bool success = strtouint32(uuid_char, &ranging_mode);
//
//    if (success && set_twr_mode(ranging_mode)) {
//        printf("OK \r\n");
//    } else {
//        printf("TWR mode parameter input error \r\n");
//    }
//}
//
//static void at_command_ledmode(const char *message) {
//    char buf[100];
//    strcpy(buf, message);
//    char *uuid_char = strtok(buf, " ");
//    uuid_char = strtok(NULL, " ");
//    uint32_t led_mode;
//    bool success = strtouint32(uuid_char, &led_mode);
//
//    if (led_mode < 0 || led_mode > 1) {
//        printf("LED mode parameter input error \r\n");
//    } else {
//        writeFlashID(led_mode, 9);
//        leds_mode = led_mode;
//        // Turn off all LEDs
//        if (leds_mode == 1) {
//            bsp_board_leds_off();
//            dwt_setleds(DWT_LEDS_DISABLE);
//        }
//
//        // Turn on LEDs from flash records
//        if (leds_mode == 0) {
//            bsp_board_led_on(BSP_BOARD_LED_0);
//            dwt_setleds(DWT_LEDS_ENABLE);
//
//            uint32_t state = getFlashID(2);
//            if ((state == 1) || (state == 2)) {
//                bsp_board_led_on(BSP_BOARD_LED_1);
//            }
//            if (state == 2) {
//                bsp_board_led_on(BSP_BOARD_LED_2);
//            }
//        }
//
//        printf("OK \r\n");
//    }
//}
//
//static void at_command_reboot(UNUSED const char *message) {
//    printf("OK \r\n");
//    // Reboot device
//    sd_nvic_SystemReset();
//}
//
//static void at_command_pwramp(const char *message) {
//
//    printf("Not Implemented \r\n");
//}
//
//static struct cmd_info commands[] = {{"STARTUWB", 8, at_cmd_start_uwb},
//                                     {"STOPUWB", 7, at_command_stop_uwb},
//                                     {"STARTBLE", 8, at_command_startable},
//                                     {"STOPBLE", 7, at_command_stopable},
//                                     {"ID", 2, at_command_id},
//                                     {"BOOTMODE", 8, at_command_bootmode},
//                                     {"RATE", 4, at_command_rate},
//                                     {"CHANNEL", 7, at_command_channel},
//                                     {"RESET", 5, at_command_reset},
//                                     {"TIMEOUT", 7, at_command_timeout},
//                                     {"TXPOWER", 7, at_command_txpower},
//                                     {"STREAMMODE", 10, at_command_streammode},
//                                     {"TWRMODE", 7, at_command_twrmode},
//                                     {"LEDMODE", 7, at_command_ledmode},
//                                     {"REBOOT", 6, at_command_reboot},
//                                     {"PWRAMP", 6, at_command_pwramp},
//                                     {NULL, 0, NULL}};
//
///**
// * @brief Task to receive UART message from freertos UART queue and parse
// *
// * @param[in] pvParameter   Pointer that will be used as the parameter for the
// * task.
// */
//void uart_task_function(void *pvParameter) {
//    bool found = false;
//    UNUSED_PARAMETER(pvParameter);
//
//    message incoming_message = {0};
//
//    while (1) {
//        vTaskDelay(100);
//
//        if (xQueueReceive(uart_queue, &incoming_message, 0) == pdPASS) {
//
//            // Handle valid AT command begining with AT+
//            if (0 == strncmp((const char *)incoming_message.data,
//                             (const char *)"AT+", (size_t)3)) {
//                for (size_t i = 0; commands[i].cmd_length != 0; i++) {
//                    if (commands[i].command != NULL &&
//                        0 == strncmp((const char *)incoming_message.data + 3,
//                                     commands[i].command,
//                                     commands[i].cmd_length)) {
//                        if (commands[i].callback != NULL) {
//                            commands[i].callback(
//                                (const char *)incoming_message.data);
//                        } else {
//                            printf("Not Implemented \r\n");
//                        }
//                        found = true;
//                        break;
//                    }
//                }
//                if (!found) {
//                    printf("ERROR Invalid AT Command\r\n");
//                }
//                found = false;
//            }
//
//            else if (0 == strncmp((const char *)incoming_message.data,
//                                  (const char *)"AT", (size_t)2)) {
//                printf("Only input AT without + command \r\n");
//            }
//
//            else {
//                printf("Not an AT command\r\n");
//            }
//        }
//    }
//}
