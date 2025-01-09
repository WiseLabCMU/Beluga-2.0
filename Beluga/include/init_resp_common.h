/**
 * @file init_resp_common.h
 * @brief Common definitions and macros for UWB (Ultra-Wideband) ranging.
 *
 * Provides various macros, inline functions, and constants used in the context
 * of UWB ranging communication. The macros define offsets, overheads, and
 * lengths for different parts of a UWB frame, such as frame control, sequence
 * counter, UUIDs, and PAN ID. Inline functions for setting and getting UUIDs,
 * PAN ID, and timestamps in UWB frames are also included.
 *
 * @date 11/21/2024
 * @author Tom Schmitz
 */

#ifndef BELUGA_INIT_RESP_COMMON_H
#define BELUGA_INIT_RESP_COMMON_H

#include <deca_device_api.h>
#include <zephyr/kernel.h>

/**
 * The number of bytes needed for the Frame control
 */
#define FC_OVERHEAD 2

/**
 * The number of bytes needed for the sequence counter
 */
#define SEQ_CNT_OVERHEAD 1

/**
 * Number of bytes needed for the PAN ID
 */
#define PAN_ID_OVERHEAD 2

/**
 * Number of bytes needed for the destination UUID
 */
#define DEST_OVERHEAD 2

/**
 * Number of bytes needed for the source UUID
 */
#define SRCV_OVERHEAD 2

/**
 * Number of bytes needed for the function code
 */
#define FUNC_CODE_OVERHEAD 1

/**
 * Number of bytes needed for the frame checksum
 */
#define CRC_OVERHEAD 2

#if defined(CONFIG_UWB_LOGIC_CLK)
/**
 * Number of bytes needed for the logic clock
 */
#define LOGIC_CLK_OVERHEAD 4
#else
/**
 * Number of bytes needed for the logic clock
 */
#define LOGIC_CLK_OVERHEAD 0
#endif

/**
 * Number of bytes needed for a ranging header
 */
#define DW_HEADER_OVERHEAD                                                     \
    (FC_OVERHEAD + SEQ_CNT_OVERHEAD + PAN_ID_OVERHEAD + DEST_OVERHEAD +        \
     SRCV_OVERHEAD + FUNC_CODE_OVERHEAD + LOGIC_CLK_OVERHEAD)

/**
 * Minimum number of bytes needed for a UWB ranging frame
 */
#define DW_FRAME_OVERHEAD (DW_HEADER_OVERHEAD + CRC_OVERHEAD)

/**
 * The base length of a UWB ranging frame
 */
#define DW_BASE_LEN DW_HEADER_OVERHEAD

/**
 * The number of bytes needed for a timestamp
 */
#define TIMESTAMP_OVERHEAD 4

/**
 * Checks if UWB is active and forces the calling function to return -EBUSY if
 * UWB is active.
 */
#define CHECK_UWB_ACTIVE()                                                     \
    do {                                                                       \
        if (get_uwb_led_state() == LED_UWB_ON) {                               \
            return -EBUSY;                                                     \
        }                                                                      \
    } while (0)

/**
 * Stalls a task until a certain condition is met. Additionally, if curly braces
 * ({}) are used on this like a loop, then code can be ran inside like a loop
 * while the condition is not met.
 *
 * @param[in] cond The condition that needs to be met before continuing
 */
#define UWB_WAIT(cond) while (!(cond))

/**
 * The byte offset of the frame control byte field
 */
#define FC_OFFSET 0

/**
 * The byte offset of the sequence counter byte field
 */
#define SEQ_CNT_OFFSET (FC_OFFSET + FC_OVERHEAD)

/**
 * The byte offset of the PAN ID byte field
 */
#define PAN_ID_OFFSET (SEQ_CNT_OFFSET + SEQ_CNT_OVERHEAD)

/**
 * The byte offset of the destination UUID byte field
 */
#define DEST_OFFSET (PAN_ID_OFFSET + PAN_ID_OVERHEAD)

/**
 * The byte offset of the source UUID byte field
 */
#define SRC_OFFSET (DEST_OFFSET + DEST_OVERHEAD)

/**
 * The byte offset of the function code byte field
 */
#define FUNC_CODE_OFFSET (SRC_OFFSET + SRCV_OVERHEAD)

/**
 * The byte offset of the logic clock byte field
 */
#define LOGIC_CLK_OFFSET (FUNC_CODE_OFFSET + FUNC_CODE_OVERHEAD)

/**
 * The byte offset of the ranging payload
 */
#define DW_BASE_PAYLOAD_OFFSET (LOGIC_CLK_OFFSET + LOGIC_CLK_OVERHEAD)

/**
 * The polling message length
 */
#define POLL_MSG_LEN DW_FRAME_OVERHEAD

/**
 * The response message length
 */
#define RESP_MSG_LEN                                                           \
    (DW_FRAME_OVERHEAD + TIMESTAMP_OVERHEAD + TIMESTAMP_OVERHEAD)

/**
 * The final message length
 */
#define FINAL_MSG_LEN                                                          \
    (DW_FRAME_OVERHEAD + TIMESTAMP_OVERHEAD + TIMESTAMP_OVERHEAD +             \
     TIMESTAMP_OVERHEAD)

/**
 * The report message length
 */
#define REPORT_MSG_LEN (DW_FRAME_OVERHEAD + TIMESTAMP_OVERHEAD)

/**
 * Index of the polling message receive timestamp
 */
#define RESP_MSG_POLL_RX_TS_IDX DW_BASE_PAYLOAD_OFFSET

/**
 * Index of the response transmit timestamp
 */
#define RESP_MSG_RESP_TX_TS_IDX (DW_BASE_PAYLOAD_OFFSET + TIMESTAMP_OVERHEAD)

/**
 * Index of the final message transmit timestamp
 */
#define FINAL_MSG_FINAL_TX_TS_IDX                                              \
    (DW_BASE_PAYLOAD_OFFSET + TIMESTAMP_OVERHEAD + TIMESTAMP_OVERHEAD)

/**
 * The speed of light in a vacuum in m/s
 */
#define SPEED_OF_LIGHT 299702547

/**
 * UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion
 * factor. 1 uus = 512 / 499.2 ?s and 1 ?s = 499.2 * 128 dtu.
 */
#define UUS_TO_DWT_TIME 65536

/**
 * @Brief Sets the source ID byte field to the specified UUID
 * @param[in] id The new source UUID
 * @param[in,out] buf The frame buffer
 */
static inline void set_src_id(uint16_t id, uint8 *buf) {
    buf[SRC_OFFSET] = (uint8)id;
    buf[SRC_OFFSET + 1] = (uint8)(id >> 8);
}

/**
 * @Brief Sets the destination ID byte field to the specified UUID
 * @param[in] id The new destination UUID
 * @param[in,out] buf The frame buffer
 */
static inline void set_dest_id(uint16_t id, uint8 *buf) {
    buf[DEST_OFFSET] = (uint8)id;
    buf[DEST_OFFSET + 1] = (uint8)(id >> 8);
}

/**
 * @brief Retrieves the source UUID from a frame buffer
 * @param[in] buf The frame buffer
 * @return The received source UUID
 */
static inline uint16_t get_src_id(const uint8 *buf) {
    return ((uint16_t)buf[SRC_OFFSET + 1] << 8) | (uint16_t)buf[SRC_OFFSET];
}

/**
 * @brief Sets the PAN ID byte field to the specified ID
 * @param[in] id The new PAN ID
 * @param[in,out] buf The frame buffer
 */
static inline void set_pan_id(uint16_t id, uint8 *buf) {
    buf[PAN_ID_OFFSET] = (uint8)id;
    buf[PAN_ID_OFFSET + 1] = (uint8)(id >> 8);
}

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

/**
 * @brief Get the TX time-stamp in a 64-bit variable.
 *
 * @return 64-bit value of the read time-stamp.
 *
 * @note This function assumes that length of time-stamps is 40 bits, for both
 * TX and RX!
 * @note The function is faster than using memcpy(). Usage of memcpy() will
 * result in a timing error
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

/**
 * @brief Read a given timestamp value from the message. In the timestamp fields
 * of the response message, the least significant byte is at the lower address.
 *
 * @param[in] ts_field Pointer on the first byte of the timestamp field to get
 * ts timestamp value
 * @param[in] ts The received timestamp from the ts_field
 *
 * @note The function is faster than using memcpy(). Usage of memcpy() will
 * result in a timing error
 */
static inline void msg_get_ts(const uint8_t *ts_field, uint32_t *ts) {
    int i;
    *ts = 0;
    for (i = 0; i < TIMESTAMP_OVERHEAD; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

/**
 * @brief Fill a given timestamp field in the response message with the given
 * value. In the timestamp fields of the response message, the least significant
 * byte is at the lower address.
 *
 * @param[in] ts_field Pointer on the first byte of the timestamp field to fill
 * ts timestamp value
 * @param[in] ts The timestamp to fill the timestamp field with
 *
 * @note The function is faster than using memcpy(). Usage of memcpy() will
 * result in a timing error
 */
static inline void msg_set_ts(uint8 *ts_field, const uint64_t ts) {
    int i;
    for (i = 0; i < TIMESTAMP_OVERHEAD; i++) {
        ts_field[i] = (ts >> (i * 8)) & 0xFF;
    }
}

#if defined(CONFIG_UWB_LOGIC_CLK)
/**
 * Set the exchange ID byte field with the given ID
 *
 * @param[in] ts_field Pointer on the first byte of the logic clock field to
 * fill ts logic clock value
 * @param[in] ts The logic clock value
 */
#define SET_EXCHANGE_ID(ts_field, ts) msg_set_ts(ts_field, ts)

/**
 * Retrieve the logic clock value from the exchange ID byte field
 *
 * @param[in] ts_field Pointer on the first byte of the logic clock field to get
 * logic clock value
 * @param[in,out] Pointer to the logic clock variable to update with the
 * ts_field
 */
#define GET_EXCHANGE_ID(ts_field, ts) msg_get_ts(ts_field, &(ts))
#else

/**
 * Set the exchange ID byte field with the given ID
 *
 * @param[in] ts_field Pointer on the first byte of the logic clock field to
 * fill ts logic clock value
 * @param[in] ts The logic clock value
 */
#define SET_EXCHANGE_ID(ts_field, id) (void)0

/**
 * Retrieve the logic clock value from the exchange ID byte field
 *
 * @param[in] ts_field Pointer on the first byte of the logic clock field to get
 * logic clock value
 * @param[in,out] Pointer to the logic clock variable to update with the
 * ts_field
 */
#define GET_EXCHANGE_ID(ts_field, ts) (void)0
#endif // defined(CONFIG_UWB_LOGIC_CLK)

#endif // BELUGA_INIT_RESP_COMMON_H
