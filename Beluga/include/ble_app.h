/*! ----------------------------------------------------------------------------
 *  @file   ble_app.h
 *
 *  @brief  Nordic BLE advertising and scanning application codes --Header file
 *
 *  @date   2020/07
 *
 *  @author WiseLab-CMU
 */

#ifndef _BLE_APP_
#define _BLE_APP_

#include <stdbool.h>
#include <stdint.h>

#include "deca_device_api.h"

/*
 * BLE neighbor node structure
 */
typedef struct node {
    uint8 UUID;       /* node ID */
    int8_t RSSI;      /* node RSSI value */
    int time_stamp;   /* Last timestamp updated ranging value */
    float range;      /* Last updated ranging value */
    int update_flag;  /* Flag to indicate the ranging value is updated or not, 1
                         if the node get updated */
    int polling_flag; /* Flag to indicate the node is passive or not, 1 if the
                         node will init uwb signal*/
    uint64_t
        ble_time_stamp; /* Last timestamp get the BLE package from this node */
} node;

/*
 * Maximum BLE neighbors in the list
 * NOTE: Change the size based on the network size
 */
#define MAX_ANCHOR_COUNT 12
extern node seen_list[MAX_ANCHOR_COUNT];

int32_t init_bt_stack(void);

#endif