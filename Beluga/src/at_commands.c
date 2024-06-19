/** @file   at_commands.c
 *
 *  @brief  Beluga AT commands
 *
 *  @date   2024/06
 *
 *  @author WiseLab-CMU
 */

// BLE Headers
#include "app_timer.h"
#include "ble_advdata.h"
#include "ble_cus.h"
#include "ble_gap.h"
#include "ble_gatt.h"
#include "ble_types.h"
#include "nrf_drv_wdt.h"

#include "nrf_fstorage_sd.h"
#include "nrf_soc.h"

#include "bsp.h"

#include "FreeRTOS.h"
#include "deca_types.h"
#include "init_main.h"

#include "at_commands.h"
#include "ble_app.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "uart.h"
#include <ctype.h>
#include <errno.h>
#include <string.h>

#include "flash.h"

// TODO: Replace strcpy with strncpy

#define UNUSED __attribute__((unused))

struct cmd_info {
    const char *command;
    size_t cmd_length;
    void (*callback)(const char *message);
};

extern ble_uuid_t m_adv_uuids[2];

QueueHandle_t uart_queue;
SemaphoreHandle_t sus_resp, sus_init, print_list_sem;

static int uwb_started;
extern int ble_started;

int leds_mode;
static uwb_lna_status = 0;
static uwb_pa_status = 0;

static bool strtouint32(const char *str, uint32_t *result) {
    char *endptr;
    unsigned long ret;

    errno = 0;
    ret = strtoul(str, &endptr, 10);

    if (errno == ERANGE || (uint64_t)ret > (uint64_t)UINT32_MAX ||
        isgraph(*endptr)) {
        *result = 0;
        return false;
    }

    *result = (uint32_t)ret;
    return true;
}

void init_at_commands(void) {
    sus_resp = xSemaphoreCreateBinary();
    sus_init = xSemaphoreCreateBinary();
    print_list_sem = xSemaphoreCreateBinary();
    uart_queue = xQueueCreate(25, sizeof(struct message));
}

void at_start_uwb(void) {
    uwb_started = 1;
    xSemaphoreGive(sus_resp);
    xSemaphoreGive(sus_init);

    if (leds_mode == 0) {
        bsp_board_led_on(BSP_BOARD_LED_2);
    }
}

void at_start_ble(void) {
    ble_started = 1;
    xSemaphoreGive(print_list_sem);
    adv_scan_start();
    if (leds_mode == 0) {
        bsp_board_led_on(BSP_BOARD_LED_1);
    }
}

void at_change_id(uint32_t id) {
    NODE_UUID = id;
    m_adv_uuids[1].uuid = NODE_UUID;

    // Change the advertising name on BLE package
    char name[] = "BN ";
    char id_attach[2];
    itoa(NODE_UUID, id_attach, 10);
    strncat(name, id_attach, strlen(id_attach));
    gap_params_init(name);
}

uint32_t at_update_advertising_info(bool init) {
    ble_advdata_t advdata;
    uint32_t retVal;

    memset(&advdata, 0, sizeof(advdata));

    if (init) {
        advdata.uuids_complete.uuid_cnt =
            sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        advdata.uuids_complete.p_uuids = m_adv_uuids;
    }

    advdata.name_type = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    return ble_advdata_set(&advdata, NULL);
}

static void at_cmd_start_uwb(UNUSED const char *message) {
    at_start_uwb();
    printf("OK \r\n");
}

static void at_command_stop_uwb(UNUSED const char *message) {
    uwb_started = 0;
    // Take UWB suspension semaphore
    xSemaphoreTake(sus_resp, portMAX_DELAY);
    xSemaphoreTake(sus_init, portMAX_DELAY);
    if (leds_mode == 0) {
        bsp_board_led_off(BSP_BOARD_LED_2);
    }
    printf("OK \r\n");
}

static void at_command_startable(UNUSED const char *message) {
    ble_started = 1;
    // Give print list semaphore
    xSemaphoreGive(print_list_sem);
    adv_scan_start();
    if (leds_mode == 0) {
        bsp_board_led_on(BSP_BOARD_LED_1);
    }
    printf("OK \r\n");
}

static void at_command_stopable(UNUSED const char *message) {
    ble_started = 0;
    sd_ble_gap_adv_stop();
    sd_ble_gap_scan_stop();
    // Take print list semaphore to stop printing
    xSemaphoreTake(print_list_sem, portMAX_DELAY);
    if (leds_mode == 0) {
        bsp_board_led_off(BSP_BOARD_LED_1);
    }
    printf("OK \r\n");
}

static void at_command_id(const char *message) {
    char buf[100];
    strcpy(buf, message);
    char *uuid_char = strtok(buf, " ");
    uuid_char = strtok(NULL, " ");
    uint32_t rec_uuid;
    bool success = strtouint32(uuid_char, &rec_uuid);

    if (!success || rec_uuid == 0) {
        printf("Invalid ID \r\n");
        return;
    }

    at_change_id(rec_uuid);
    at_update_advertising_info(false);

    writeFlashID(rec_uuid, 1);
    printf("OK\r\n");
}

static void at_command_bootmode(const char *message) {
    char buf[100];
    strcpy(buf, message);
    char *uuid_char = strtok(buf, " ");
    uuid_char = strtok(NULL, " ");
    uint32_t mode;
    bool success = strtouint32(uuid_char, &mode);

    if (mode < 0 || mode > 2 || !success) {
        printf("Invalid bootmode parameter \r\n");
    } else {
        if (mode == 1) {
            writeFlashID(1, 2);
        } else if (mode == 2) {
            writeFlashID(2, 2);
        } else {
            writeFlashID(0, 2);
        }

        printf("Bootmode: %d OK \r\n", mode);
    }
}

static void at_command_rate(const char *message) {
    char buf[100];
    strcpy(buf, message);
    char *uuid_char = strtok(buf, " ");
    uuid_char = strtok(NULL, " ");
    uint32_t rate;
    bool success = strtouint32(uuid_char, &rate);

    if (rate < 0 || rate > 500 || !success) {
        printf("Invalid rate parameter \r\n");
    } else {
        writeFlashID(rate, 3);
        set_initiator_freq(rate);

        // reconfig BLE advertising data
        if (rate == 0) {
            advertising_reconfig(0);
        } else {
            advertising_reconfig(1);
        }

        printf("Rate: %d OK \r\n", rate);
    }
}

static void at_command_channel(const char *message) {
    char buf[100];
    strcpy(buf, message);
    char *uuid_char = strtok(buf, " ");
    uuid_char = strtok(NULL, " ");
    uint32_t channel;
    bool success = strtouint32(uuid_char, &channel);

    if (success && set_uwb_pgdelay(channel)) {
        printf("OK\r\n");
    } else {
        printf("Invalid Channel number \r\n");
    }
}

static void _at_helper_delete_record(uint16_t fileID, uint16_t recordKey) {
    fds_record_desc_t record_desc;
    fds_find_token_t ftok;
    memset(&ftok, 0x00, sizeof(fds_find_token_t));
    if (fds_record_find(fileID, recordKey, &record_desc, &ftok) ==
        FDS_SUCCESS) {
        ret_code_t ret = fds_record_delete(&record_desc);
        if (ret != FDS_SUCCESS) {
            printf("FDS Delete error \r\n");
        }
    }
}

static void at_command_reset(UNUSED const char *message) {
    // Delete ID record
    _at_helper_delete_record(FILE_ID, RECORD_KEY_1);

    // Delete rate record
    _at_helper_delete_record(FILE_ID, RECORD_KEY_3);

    // Delete channel record
    _at_helper_delete_record(FILE_ID, RECORD_KEY_4);

    // Delete timeout record
    _at_helper_delete_record(FILE_ID, RECORD_KEY_5);

    // Delete Tx power record
    _at_helper_delete_record(FILE_ID, RECORD_KEY_6);

    // Delete stream mode record
    _at_helper_delete_record(FILE_ID, RECORD_KEY_7);

    // Delete TWR mode record
    _at_helper_delete_record(FILE_ID, RECORD_KEY_8);

    // Delete LED mode record
    _at_helper_delete_record(FILE_ID, RECORD_KEY_9);

    // Delete BOOT mode record
    _at_helper_delete_record(FILE_ID, RECORD_KEY_2);

    printf("Reset OK \r\n");
}

static void at_command_timeout(const char *message) {
    char buf[100];
    strcpy(buf, message);
    char *uuid_char = strtok(buf, " ");
    uuid_char = strtok(NULL, " ");
    uint32_t timeout;
    bool success = strtouint32(uuid_char, &timeout);
    // printf("%d \r\n", timeout);

    if (!success) {
        printf("Timeout cannot be negative \r\n");
    } else {
        writeFlashID(timeout, 5);
        set_time_out(timeout);

        printf("OK \r\n");
    }
}

static void at_command_txpower(const char *message) {
    char buf[100];
    strcpy(buf, message);
    char *uuid_char = strtok(buf, " ");
    uuid_char = strtok(NULL, " ");
    uint32_t tx_power;
    bool success = strtouint32(uuid_char, &tx_power);
    // printf("%d \r\n", timeout);

    if (success && set_uwb_tx_power(tx_power)) {
        printf("OK \r\n");
    } else {
        printf("Tx Power parameter input error \r\n");
    }
}

static void at_command_streammode(const char *message) {
    char buf[100];
    strcpy(buf, message);
    char *uuid_char = strtok(buf, " ");
    uuid_char = strtok(NULL, " ");
    uint32_t stream_mode;
    bool success = strtouint32(uuid_char, &stream_mode);

    if (success && set_streaming_mode(stream_mode)) {
        printf("OK \r\n");
    } else {
        printf("Stream mode parameter input error \r\n");
    }
}

static void at_command_twrmode(const char *message) {
    char buf[100];
    strcpy(buf, message);
    char *uuid_char = strtok(buf, " ");
    uuid_char = strtok(NULL, " ");
    uint32_t ranging_mode;
    bool success = strtouint32(uuid_char, &ranging_mode);

    if (success && set_twr_mode(ranging_mode)) {
        printf("OK \r\n");
    } else {
        printf("TWR mode parameter input error \r\n");
    }
}

static void at_command_ledmode(const char *message) {
    char buf[100];
    strcpy(buf, message);
    char *uuid_char = strtok(buf, " ");
    uuid_char = strtok(NULL, " ");
    uint32_t led_mode;
    bool success = strtouint32(uuid_char, &led_mode);

    if (led_mode < 0 || led_mode > 1) {
        printf("LED mode parameter input error \r\n");
    } else {
        writeFlashID(led_mode, 9);
        leds_mode = led_mode;
        // Turn off all LEDs
        if (leds_mode == 1) {
            bsp_board_leds_off();
            dwt_setleds(DWT_LEDS_DISABLE);
        }

        // Turn on LEDs from flash records
        if (leds_mode == 0) {
            bsp_board_led_on(BSP_BOARD_LED_0);
            dwt_setleds(DWT_LEDS_ENABLE);

            uint32_t state = getFlashID(2);
            if ((state == 1) || (state == 2)) {
                bsp_board_led_on(BSP_BOARD_LED_1);
            }
            if (state == 2) {
                bsp_board_led_on(BSP_BOARD_LED_2);
            }
        }

        printf("OK \r\n");
    }
}

static void at_command_reboot(UNUSED const char *message) {
    printf("OK \r\n");
    // Reboot device
    sd_nvic_SystemReset();
}

static void at_command_pwramp(const char *message) {

    printf("Not Implemented \r\n");
}

static struct cmd_info commands[] = {{"STARTUWB", 8, at_cmd_start_uwb},
                                     {"STOPUWB", 7, at_command_stop_uwb},
                                     {"STARTBLE", 8, at_command_startable},
                                     {"STOPBLE", 7, at_command_stopable},
                                     {"ID", 2, at_command_id},
                                     {"BOOTMODE", 8, at_command_bootmode},
                                     {"RATE", 4, at_command_rate},
                                     {"CHANNEL", 7, at_command_channel},
                                     {"RESET", 5, at_command_reset},
                                     {"TIMEOUT", 7, at_command_timeout},
                                     {"TXPOWER", 7, at_command_txpower},
                                     {"STREAMMODE", 10, at_command_streammode},
                                     {"TWRMODE", 7, at_command_twrmode},
                                     {"LEDMODE", 7, at_command_ledmode},
                                     {"REBOOT", 6, at_command_reboot},
                                     {"PWRAMP", 6, at_command_pwramp},
                                     {NULL, 0, NULL}};

/**
 * @brief Task to receive UART message from freertos UART queue and parse
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the
 * task.
 */
void uart_task_function(void *pvParameter) {
    bool found = false;
    UNUSED_PARAMETER(pvParameter);

    message incoming_message = {0};

    while (1) {
        vTaskDelay(100);

        if (xQueueReceive(uart_queue, &incoming_message, 0) == pdPASS) {

            // Handle valid AT command begining with AT+
            if (0 == strncmp((const char *)incoming_message.data,
                             (const char *)"AT+", (size_t)3)) {
                for (size_t i = 0; commands[i].cmd_length != 0; i++) {
                    if (commands[i].command != NULL &&
                        0 == strncmp((const char *)incoming_message.data + 3,
                                     commands[i].command,
                                     commands[i].cmd_length)) {
                        if (commands[i].callback != NULL) {
                            commands[i].callback(
                                (const char *)incoming_message.data);
                        } else {
                            printf("Not Implemented \r\n");
                        }
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    printf("ERROR Invalid AT Command\r\n");
                }
                found = false;
            }

            else if (0 == strncmp((const char *)incoming_message.data,
                                  (const char *)"AT", (size_t)2)) {
                printf("Only input AT without + command \r\n");
            }

            else {
                printf("Not an AT command\r\n");
            }
        }
    }
}
