/** @file   at_commands.h
 *
 *  @brief  Beluga AT commands
 *
 *  @date   2024/06
 *
 *  @author WiseLab-CMU
 */

#ifndef AT_COMMANDS_H
#define AT_COMMANDS_H

#include <stdint.h>
#include <unistd.h>

void init_at_commands(void);
void at_start_uwb(void);
void at_start_ble(void);
void at_change_id(uint32_t id);
uint32_t at_update_advertising_info(bool init);

/**
 * @brief Task to receive UART message from freertos UART queue and parse
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
void uart_task_function(void * pvParameter);

extern int leds_mode;
extern SemaphoreHandle_t print_list_sem;
extern const size_t ble_uuid_cnt;

#endif