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

#include <stdint.h>
#include <stdbool.h>

#include "deca_device_api.h"

/*
 * BLE neighbor node structure
 */
typedef struct node{
    uint8 UUID;         /* node ID */
    int8_t RSSI;        /* node RSSI value */
    int time_stamp;     /* Last timestamp updated ranging value */
    float range;        /* Last updated ranging value */
    int update_flag;    /* Flag to indicate the ranging value is updated or not, 1 if the node get updated */
    int polling_flag;   /* Flag to indicate the node is passive or not, 1 if the node will init uwb signal*/
    int ble_time_stamp; /* Last timestamp get the BLE package from this node */
} node;

/*
 * Maximum BLE neighbors in the list
 * NOTE: Change the size based on the network size
 */
#define MAX_ANCHOR_COUNT 12

void scan_start(void);
void adv_scan_start(void);
void ble_stack_init(void);
void peer_manager_init(void);
void delete_bonds(void);
void buttons_leds_init(bool * p_erase_bonds);
void gap_params_init(char* device_name);
void gatt_init(void);
void conn_params_init(void);
void advertising_init(void);
void log_init(void);
void power_manage(void);
void advertising_reconfig(int change);
void non_connectable_advertising_init(int mode);
void services_init(void);
void change_char_value(void);
void update_char_value(uint8 UUID, float range);


#endif