/*! ----------------------------------------------------------------------------
 *  @file   flash.c
 *
 *  @brief  An implementation of modify and retrive data through flash data
 * storage
 *
 *  @date   2020/06
 *
 *  @author WiseLab-CMU
 */

#include <flash.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>

#define FILE_ID          0x0015 /* The ID of the file to write the records into. */
#define RECORD_KEY_1     0      /* A key for the first record. (ID) */
#define RECORD_KEY_2     1      /* A key for the second record. (BOOTMODE) */
#define RECORD_KEY_3     2      /* A key for the third record. (RATE)*/
#define RECORD_KEY_4     3      /* A key for the forth record. (CHANNEL)*/
#define RECORD_KEY_5     4      /* A key for the fifth record. (BLE Timeout)*/
#define RECORD_KEY_6     5      /* A key for the sixth record. (TX Power)*/
#define RECORD_KEY_7     6      /* A key for the seventh record. (STREAMMODE)*/
#define RECORD_KEY_8     7      /* A key for the eighth record. (TWRMODE)*/
#define RECORD_KEY_9     8      /* A key for the ninth record. (LEDMODE)*/

#define PARTITION        storage_partition

#define PARTITION_OFFSET FIXED_PARTITION_OFFSET(PARTITION)
#define PARTITION_DEVICE FIXED_PARTITION_DEVICE(PARTITION)

#define FLASH_PAGE_SIZE  4096

static const struct device *flash_dev = PARTITION_DEVICE;

static uint32_t id2key(uint32_t record) {
    uint32_t record_key = 0;
    switch (record) {
    case CONFIG_ID:
        record_key = RECORD_KEY_1;
        break;
    case CONFIG_BM:
        record_key = RECORD_KEY_2;
        break;
    case CONFIG_RATE:
        record_key = RECORD_KEY_3;
        break;
    case CONFIG_CHAN:
        record_key = RECORD_KEY_4;
        break;
    case CONFIG_TIME:
        record_key = RECORD_KEY_5;
        break;
    case CONFIG_TXP:
        record_key = RECORD_KEY_6;
        break;
    case CONFIG_SM:
        record_key = RECORD_KEY_7;
        break;
    case CONFIG_TWR:
        record_key = RECORD_KEY_8;
        break;
    case CONFIG_LED:
        record_key = RECORD_KEY_9;
        break;
    default:
        assert_print("Invalid record");
    }
    return record_key;
}

int32_t initFlash(void) {
    if (!device_is_ready(flash_dev)) {
        printk("Flash device is not ready\n");
        return 1;
    }
    return 0;
}

void writeFlashID(uint32_t id, int32_t record) {
    uint32_t record_key = id2key(id);
    off_t offset = PARTITION_OFFSET + (record_key << 2);

    if (flash_write(flash_dev, offset, &record, sizeof(int32_t)) != 0) {
        printk("Flash write failed!\n");
    }
}

int32_t readFlashID(uint32_t id) {
    uint32_t record_key = id2key(id);
    off_t offset = PARTITION_OFFSET + (record_key << 2);
    int32_t value;

    if (flash_read(flash_dev, offset, &value, sizeof(int32_t)) != 0) {
        printk("Flash read failed!\n");
        return 0;
    }

    return value;
}

void eraseRecords(void) {
    if (flash_erase(flash_dev, PARTITION_OFFSET, FLASH_PAGE_SIZE) != 0) {
        printk("flash erase failed!\n");
    }
}
