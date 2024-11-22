//
// Created by tom on 11/21/24.
//

#ifndef BELUGA_INIT_RESP_COMMON_H
#define BELUGA_INIT_RESP_COMMON_H

#include <deca_device_api.h>
#include <zephyr/kernel.h>

#define FC_OVERHEAD        2
#define SEQ_CNT_OVERHEAD   1
#define PAN_ID_OVERHEAD    2
#define DEST_OVERHEAD      2
#define SRCV_OVERHEAD      2
#define FUNC_CODE_OVERHEAD 1
#define CRC_OVERHEAD       2

#if IS_ENABLED(CONFIG_UWB_LOGIC_CLK)
#define LOGIC_CLK_OVERHEAD 4
#else
#define LOGIC_CLK_OVERHEAD 0
#endif

#define DW_HEADER_OVERHEAD                                                     \
    (FC_OVERHEAD + SEQ_CNT_OVERHEAD + PAN_ID_OVERHEAD + DEST_OVERHEAD +        \
     SRCV_OVERHEAD + FUNC_CODE_OVERHEAD + LOGIC_CLK_OVERHEAD)
#define DW_FRAME_OVERHEAD  (DW_HEADER_OVERHEAD + CRC_OVERHEAD)

#define DW_BASE_LEN        DW_HEADER_OVERHEAD
#define TIMESTAMP_OVERHEAD 4

#define CHECK_UWB_ACTIVE()                                                     \
    do {                                                                       \
        if (get_uwb_led_state() == LED_UWB_ON) {                               \
            return -EBUSY;                                                     \
        }                                                                      \
    } while (0)
#define UWB_WAIT(cond)         while (!(cond))

#define FC_OFFSET              0
#define SEQ_CNT_OFFSET         (FC_OFFSET + FC_OVERHEAD)
#define PAN_ID_OFFSET          (SEQ_CNT_OFFSET + SEQ_CNT_OVERHEAD)
#define DEST_OFFSET            (PAN_ID_OFFSET + PAN_ID_OVERHEAD)
#define SRC_OFFSET             (DEST_OFFSET + DEST_OVERHEAD)
#define FUNC_CODE_OFFSET       (SRC_OFFSET + SRCV_OVERHEAD)
#define LOGIC_CLK_OFFSET       (FUNC_CODE_OFFSET + FUNC_CODE_OVERHEAD)
#define DW_BASE_PAYLOAD_OFFSET (LOGIC_CLK_OFFSET + LOGIC_CLK_OVERHEAD)

#define POLL_MSG_LEN           DW_FRAME_OVERHEAD
#define RESP_MSG_LEN                                                           \
    (DW_FRAME_OVERHEAD + TIMESTAMP_OVERHEAD + TIMESTAMP_OVERHEAD)
#define FINAL_MSG_LEN                                                          \
    (DW_FRAME_OVERHEAD + TIMESTAMP_OVERHEAD + TIMESTAMP_OVERHEAD +             \
     TIMESTAMP_OVERHEAD)
#define REPORT_MSG_LEN (DW_FRAME_OVERHEAD + TIMESTAMP_OVERHEAD)

// Response messages
#define RESP_MSG_POLL_RX_TS_IDX DW_BASE_PAYLOAD_OFFSET
#define RESP_MSG_RESP_TX_TS_IDX (DW_BASE_PAYLOAD_OFFSET + TIMESTAMP_OVERHEAD)
#define FINAL_MSG_FINAL_TX_TS_IDX                                              \
    (DW_BASE_PAYLOAD_OFFSET + TIMESTAMP_OVERHEAD + TIMESTAMP_OVERHEAD)

#define SPEED_OF_LIGHT 299702547

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for
 * both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static inline uint64_t get_rx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for
 * both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static inline uint64_t get_tx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn msg_get_ts()
 *
 * @brief Read a given timestamp value from the message. In the
 * timestamp fields of the response message, the least significant byte is at
 * the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to get
 *         ts  timestamp value
 *
 * @return none
 */
static void msg_get_ts(const uint8_t *ts_field, uint32_t *ts) {
    int i;
    *ts = 0;
    for (i = 0; i < TIMESTAMP_OVERHEAD; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn msg_set_ts()
 *
 * @brief Fill a given timestamp field in the response message with the given
 * value. In the timestamp fields of the response message, the least significant
 * byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void msg_set_ts(uint8 *ts_field, const uint64 ts) {
    int i;
    for (i = 0; i < TIMESTAMP_OVERHEAD; i++) {
        ts_field[i] = (ts >> (i * 8)) & 0xFF;
    }
}

#endif // BELUGA_INIT_RESP_COMMON_H
