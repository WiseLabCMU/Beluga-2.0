/**
 * @file beluga_service.h
 *
 * @brief
 *
 * @date 5/1/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_DTS_BELUGA_SERVICE_H
#define BELUGA_DTS_BELUGA_SERVICE_H

#include <zephyr/types.h>
#include <zephyr/bluetooth/uuid.h>
#include <services/beluga_service_common.h>

#define BT_UUID_BELUGA_SVC_VAL BT_UUID_128_ENCODE(0x89b468da, 0x535a, 0x4268, 0xa703, 0x87ddc32450b8)
#define BT_UUID_BELUGA_RANGE_VAL BT_UUID_128_ENCODE(0x89b468db, 0x535a, 0x4268, 0xa703, 0x87ddc32450b8)
#define BT_UUID_BELUGA_SYNC_VAL BT_UUID_128_ENCODE(0x89b468dc, 0x535a, 0x4268, 0xa703, 0x87ddc32450b8)

#define BT_UUID_BELUGA_SVC BT_UUID_DECLARE_128(BT_UUID_BELUGA_SVC_VAL)
#define BT_UUID_BELUGA_RANGE BT_UUID_DECLARE_128(BT_UUID_BELUGA_RANGE_VAL)
#define BT_UUID_BELUGA_SYNC BT_UUID_DECLARE_128(BT_UUID_BELUGA_SYNC_VAL)

#endif //BELUGA_DTS_BELUGA_SERVICE_H
