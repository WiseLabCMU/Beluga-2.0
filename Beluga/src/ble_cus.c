/*! ----------------------------------------------------------------------------
 *  @file   ble_cus.c
 *
 *  @brief  Nordic BLE custom service codes
 *
 *  @date   2020/08
 *
 *  @author WiseLab-CMU
 *
 *  NOTE: Part of the codes are reference from
 * https://github.com/bjornspockeli/custom_ble_service_example
 */

#include "ble_cus.h"
#include "sdk_config.h"
#include <string.h>

/**@brief   Maximum length of data (in bytes) that can be transmitted to the
 * peer */
// NOTE: Modify NRF_SDH_BLE_GATT_MAX_MTU_SIZE value in sdk_config.h file to
// increase tranferable data length
#define BLE_MAX_DT_CHAR_LEN                                                    \
    NRF_SDH_BLE_GATT_MAX_MTU_SIZE /**< Maximum length of the Characteristic    \
                                     (in bytes). */

/**@brief Function for adding data transmission characteristic.
 *
 * @param[in] p_nus       Nordic custom service structure.
 * @param[in] p_nus_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t dt_char_add(ble_cus_t *p_nus,
                            const ble_cus_init_t *p_nus_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_md_t cccd_md;

    // Configure cccd
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    // Configure characteristic metadata attribute
    memset(&char_md, 0, sizeof(char_md));
    char_md.p_cccd_md = &cccd_md;
    char_md.char_props.notify = 1;
    char_md.char_props.read = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_sccd_md = NULL;

    // Set characteristic UUID from custom base UUID
    ble_uuid.type = p_nus->uuid_type_char;
    ble_uuid.uuid = CUSTOM_DT_UUID;

    // Configure attribute metadata
    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 1;

    // Configure characteristic value
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = 0;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len =
        BLE_MAX_DT_CHAR_LEN; // Maximum data length can be transmitted

    return sd_ble_gatts_characteristic_add(
        p_nus->service_handle, &char_md, &attr_char_value, &p_nus->dt_handles);
}

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
uint32_t ble_cus_init(ble_cus_t *p_cus, const ble_cus_init_t *p_cus_init) {
    if (p_cus == NULL || p_cus_init == NULL) {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid_t ble_uuid_char;

    // Initialize service structure
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add custom service base UUID for service
    // NOTE: After adding a new base UUID, the sdk_config.h file should be
    // modified
    ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CUSTOM_SERVICE_UUID;

    // Add second custom service bae UUID for characteristic
    ble_uuid128_t base_uuid_char = {CUSTOM_DT_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid_char, &p_cus->uuid_type_char);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    ble_uuid_char.type = p_cus->uuid_type_char;
    ble_uuid_char.uuid = CUSTOM_DT_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid,
                                        &p_cus->service_handle);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // Add the data transmission characteristic.
    err_code = dt_char_add(p_cus, p_cus_init);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    return NRF_SUCCESS;
}
