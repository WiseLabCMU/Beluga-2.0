/*! ----------------------------------------------------------------------------
 *  @file   ble_cus.h
 *
 *  @brief  Nordic BLE custom service codes --Header file
 *
 *  @date   2020/08
 *
 *  @author WiseLab-CMU
 *
 *  NOTE: Part of the codes are reference from
 * https://github.com/bjornspockeli/custom_ble_service_example
 */

#ifndef BLE_CUS_H
#define BLE_CUS_H

#include "ble_gatts.h"
#include "ble_srv_common.h"
#include "stdint.h"

#define CUSTOM_SERVICE_UUID_BASE                                               \
    {                                                                          \
        0xe7, 0x29, 0x13, 0xc2, 0xa1, 0xba, 0x11, 0x9c, 0x1f, 0x4c, 0x46,      \
            0xc9, 0xd9, 0x21, 0x0c, 0x68                                       \
    }
#define CUSTOM_DT_UUID_BASE                                                    \
    {                                                                          \
        0x37, 0x9a, 0xb8, 0x89, 0xc8, 0x7e, 0x56, 0xab, 0x3d, 0x4b, 0x34,      \
            0xc6, 0xf2, 0xbd, 0x3b, 0x00                                       \
    }
#define CUSTOM_SERVICE_UUID 0x21d9
#define CUSTOM_DT_UUID      0xbdf2

#define BLE_CUS_DEF(_name)  static ble_cus_t _name;

/**@brief Custom Service init structure. This contains all options and data
 * needed for initialization of the service.*/
typedef struct {
    uint8_t initial_custom_value; /**< Initial custom value */
    ble_srv_cccd_security_mode_t
        custom_value_char_attr_md; /**< Initial security level for Custom
                                      characteristics attribute */
} ble_cus_init_t;

/**@brief Custom Service structure. This contains various status information for
 * the service. */
struct ble_cus_s {
    uint8_t uuid_type;       /**< UUID type for Nordic service Base UUID. */
    uint8_t uuid_type_char;  /**< UUID type for Nordic specific characteristic
                                UUID. */
    uint16_t service_handle; /**< Handle of Custom Service (as provided by the
                                BLE stack). */
    ble_gatts_char_handles_t
        dt_handles; /**< Handles related to the RX characteristic (as provided
                       by the SoftDevice). */
    uint16_t conn_handle; /**< Handle of the current connection (as provided by
                             the BLE stack, is BLE_CONN_HANDLE_INVALID if not in
                             a connection). */
};

// Forward declaration of the ble_cus_t type.
typedef struct ble_cus_s ble_cus_t;

/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have
 * to be supplied by the application. It will be initialized by this function,
 * and will later be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise
 * an error code.
 */
uint32_t ble_cus_init(ble_cus_t *p_cus, const ble_cus_init_t *p_cus_init);

#endif
