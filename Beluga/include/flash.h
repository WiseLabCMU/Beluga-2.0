/*! ----------------------------------------------------------------------------
 *  @file   flash.h
 *
 *  @brief  An implementation of modify and retrive data through flash data
 * storage --Header file
 *
 *  @date   2020/06
 *
 *  @author WiseLab-CMU
 */

#ifndef _FLASH_H_
#define _FLASH_H_

#include <stdint.h>

#define CONFIG_ID   1 /* Idenfication for ID configuration */
#define CONFIG_BM   2 /* Idenfication for Boot mode configuration */
#define CONFIG_RATE 3 /* Idenfication for polling rate configuration */
#define CONFIG_CHAN 4 /* Idenfication for UWB channel configuration */
#define CONFIG_TIME 5 /* Idenfication for BLE timeout configuration */
#define CONFIG_TXP  6 /* Idenfication for UWB TX power configuration */
#define CONFIG_SM   7 /* Idenfication for streammode configuration */
#define CONFIG_TWR                                                             \
    8 /* Idenfication for two way raning scheme mode configuration */
#define CONFIG_LED 9 /* Idenfication for LED mode configuration */

int32_t initFlash(void);
void writeFlashID(uint32_t id, int32_t record);
int32_t readFlashID(uint32_t id);
void eraseRecords(void);

#endif