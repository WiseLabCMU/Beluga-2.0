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

#define FILE_ID      0x0015 /* The ID of the file to write the records into. */
#define RECORD_KEY_1 32     /* A key for the first record. (ID) */
#define RECORD_KEY_2                                                           \
    (RECORD_KEY_1 + 1) /* A key for the second record. (BOOTMODE) */
#define RECORD_KEY_3                                                           \
    (RECORD_KEY_1 + 2) /* A key for the third record.                          \
                          (RATE)*/
#define RECORD_KEY_4                                                           \
    (RECORD_KEY_1 + 3) /* A key for the forth record. (CHANNEL)*/
#define RECORD_KEY_5                                                           \
    (RECORD_KEY_1 + 4) /* A key for the fifth record. (BLE Timeout)*/
#define RECORD_KEY_6                                                           \
    (RECORD_KEY_1 + 5) /* A key for the sixth record. (TX Power)*/
#define RECORD_KEY_7                                                           \
    (RECORD_KEY_1 + 6) /* A key for the seventh record. (STREAMMODE)*/
#define RECORD_KEY_8                                                           \
    (RECORD_KEY_1 + 7) /* A key for the eighth record. (TWRMODE)*/
#define RECORD_KEY_9                                                           \
    (RECORD_KEY_1 + 8) /* A key for the ninth record. (LEDMODE)*/

#define MAX_RECORD_ID    (CONFIG_LED + 1)

#define PARTITION        storage_partition

#define PARTITION_OFFSET FIXED_PARTITION_OFFSET(PARTITION)
#define PARTITION_DEVICE FIXED_PARTITION_DEVICE(PARTITION)

#define FLASH_PAGE_SIZE  4096

#define FLASH_FALSE      INT32_C(0x1111)
#define FLASH_TRUE       INT32_C(0xFFFF)
#define FLASH_UNWRITTEN  INT32_C(-1)

static const struct device *flash_dev = PARTITION_DEVICE;
static bool internal_read = false;

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

void read_stored_values(int32_t *records) {
    internal_read = true;
    for (size_t i = 1; i < MAX_RECORD_ID; i++) {
        records[i - 1] = readFlashID(i);
    }
    internal_read = false;
}

void restore_records(const int32_t *records) {
    uint32_t record_key;

    for (size_t i = 1; i < MAX_RECORD_ID; i++) {
        if (records[i - 1] == FLASH_FALSE || records[i - 1] == FLASH_TRUE) {
            record_key = id2key((uint32_t)i);
            off_t offset = PARTITION_OFFSET + (record_key << 2);

            if (flash_write(flash_dev, offset, &records[i - 1],
                            sizeof(int32_t)) != 0) {
                printk("Flash write failed!\n");
            }
            printk("   Attempted to write %x at 0x%x\n", records[i - 1],
                   offset);

            internal_read = true;
            int32_t readValue = readFlashID((uint32_t)i);
            internal_read = false;
            if (records[i - 1] == readValue) {
                printk("Flash OK\n");
            } else {
                printk("Flash failed:\n");
                printk("Write value %x does not match read value %x\n",
                       records[i - 1], readValue);
            }
        }
    }
}

void writeFlashID(uint32_t id, int32_t record) {
    int32_t records[MAX_RECORD_ID - 1];

    if (id >= MAX_RECORD_ID) {
        return;
    }

    if (record == 0) {
        record = FLASH_FALSE;
    } else if (record == 1) {
        record = FLASH_TRUE;
    } else if (id != CONFIG_ID) {
        return;
    }

    read_stored_values(records);
    eraseRecords();
    records[id - 1] = record;
    restore_records(records);
}

int32_t readFlashID(uint32_t id) {
    uint32_t record_key = id2key(id);
    off_t offset = PARTITION_OFFSET + (record_key << 2);
    int32_t value = 0;

    printk("   Attempted to read 0x%x\n", offset);
    if (flash_read(flash_dev, offset, &value, sizeof(int32_t)) != 0) {
        printk("Flash read failed!\n");
        return -1;
    }
    printk("   Data read: %x\n", value);

    if (!internal_read) {
        if (value == FLASH_TRUE) {
            value = 1;
        } else if (value == FLASH_FALSE) {
            value = 0;
        }
    }

    return value;
}

void eraseRecords(void) {
    if (flash_erase(flash_dev, PARTITION_OFFSET, FLASH_PAGE_SIZE) != 0) {
        printk("flash erase failed!\n");
    }
}
