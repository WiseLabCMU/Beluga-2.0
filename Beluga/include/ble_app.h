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
#include <zephyr/kernel.h>

#include "deca_device_api.h"

/*
 * BLE neighbor node structure
 */
struct node {
    uint16_t UUID;      /* node ID */
    int8_t RSSI;        /* node RSSI value */
    int64_t time_stamp; /* Last timestamp updated ranging value */
    float range;        /* Last updated ranging value */
    int update_flag;  /* Flag to indicate the ranging value is updated or not, 1
                         if the node get updated */
    int polling_flag; /* Flag to indicate the node is passive or not, 1 if the
                         node will init uwb signal*/
    int64_t
        ble_time_stamp; /* Last timestamp get the BLE package from this node */

#if IS_ENABLED(CONFIG_UWB_LOGIC_CLK)
    uint32_t exchange_id; /* Ranging exchange ID (Logic clock) */
#endif
};

#define MAX_ANCHOR_COUNT CONFIG_BELUGA_NETWORK_SIZE

extern struct node seen_list[MAX_ANCHOR_COUNT];

int32_t init_bt_stack(void);
int32_t deinit_bt_stack(void);
int32_t enable_bluetooth(void);
int32_t disable_bluetooth(void);
void update_node_id(uint16_t uuid);
uint16_t get_NODE_UUID(void);
void advertising_reconfig(int32_t change);
bool check_ble_enabled(void);

bool save_and_disable_bluetooth(void);
void restore_bluetooth(bool state);

#if defined(CONFIG_BELUGA_GATT)
void update_ble_service(uint16_t uuid, float range)
#else
#define update_ble_service(x, y) (void)0
#endif // defined(CONFIG_BELUGA_GATT)

#endif