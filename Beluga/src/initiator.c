/**
 * @file initiator.c
 *
 * @brief This module implements the functionality for the initiator role in a
 * UWB-based ranging system.
 *
 * The primary objective of this module is to perform the communication sequence
 * required to initiate a ranging process and compute the distance between the
 * initiator and a responder in a UWB-based system.
 *
 * The module handles:
 * - Sending a poll message to the responder.
 * - Receiving a response from the responder.
 * - Sending a final message containing timestamp data (double-sided
 * ranging only).
 * - Receiving a report message containing the time-of-flight (ToF) data, which
 * is used to calculate the distance (double-sided ranging only).
 *
 * @date 6/1/2024
 *
 * @author Decawave
 * @author Tome Schmitz
 */

#include <app_leds.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <init_resp_common.h>
#include <initiator.h>
#include <port_platform.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/**
 * Logger for the initiator module
 */
LOG_MODULE_REGISTER(initializer_logger, LOG_LEVEL_INF);

/**
 * Semaphore for suspending the initiator task
 */
K_SEM_DEFINE(k_sus_init, 0, 1);

/**
 * @brief Frames used in the ranging process
 *
 * The frames used here are Decawave specific ranging frames complying with the
 * IEEE 802.15.4 standard data frame encoding. The frames are the following:
 * - a poll message sent by the initiator to trigger the ranging exchange.
 * - a response message sent by the responder allowing the initiator to go on
 * with the process or to complete the exchange and provide all information
 * needed by the initiator to compute the time-of-flight (distance) estimate.
 * - a final message sent by the initiator to complete the exchange and provide
 * all information needed by the responder to compute the time-of-flight
 * estimate.
 * - a report message sent by the responder telling the initiator what the
 * time-of-flight estimate is.
 * .
 *
 * All messages end with a 2-byte checksum automatically set by DW1000.
 *
 * @{
 */
static uint8 tx_poll_msg[POLL_MSG_LEN] = {0x41, 0x88, 0,   0xCA, 0xDE, 'W',
                                          'A',  'V',  'E', 0x61, 0,    0};
static uint8 rx_resp_msg[RESP_MSG_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x50,
    0,    0,    0, 0,    0,    0,   0,   0,   0,   0};
static uint8 tx_final_msg[FINAL_MSG_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x69, 0, 0,
    0,    0,    0, 0,    0,    0,   0,   0,   0,   0,    0, 0};
static uint8 rx_report_msg[REPORT_MSG_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE3, 0, 0, 0, 0, 0, 0};

/**
 * @}
 */

/**
 * The maximum length of the receive buffer
 */
#define RX_BUF_LEN MAX(RESP_MSG_LEN, REPORT_MSG_LEN)

/**
 * The buffer where received data is stored
 */
static uint8 rx_buffer[RX_BUF_LEN];

/**
 * Multiplication factor to convert carrier integrator value to a frequency
 * offset in Hertz.
 */
static double freq_offset_multiplier = FREQ_OFFSET_MULTIPLIER;

/**
 * Multiplication factor to convert frequency offset in Hertz to PPM crystal
 * offset.
 */
static double hertz_to_ppm_multiplier = HERTZ_TO_PPM_MULTIPLIER_CHAN_5;

/**
 * This is the delay from Frame RX timestamp to TX reply timestamp used for
 * calculating/setting the DW1000's delayed TX function.
 */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2000

/**
 * This is the delay from the end of the frame transmission to the enable of the
 * receiver, as programmed for the DW1000's wait for response feature.
 */
#define POLL_TX_TO_RESP_RX_DLY_UUS 300

/**
 * @brief Sets the source IDs for the messages that the initiator sends and the
 * destination ID for the messages the initiator receives
 *
 * @param[in] id The ID of the node
 *
 * @return 0 upon success
 * @return -EBUSY if UWB is active
 */
int set_initiator_id(uint16_t id) {
    CHECK_UWB_ACTIVE();

    set_src_id(id, tx_poll_msg);
    set_dest_id(id, rx_resp_msg);
    set_src_id(id, tx_final_msg);
    set_dest_id(id, rx_report_msg);

    return 0;
}

/**
 * @brief Sets the personal area network (PAN) ID for the initiator messages
 *
 * @param[in] id
 * @return 0 upon success
 * @return -EBUSY if UWB is active
 */
int set_initiator_pan_id(uint16_t id) {
    CHECK_UWB_ACTIVE();

    set_pan_id(id, tx_poll_msg);
    set_pan_id(id, rx_resp_msg);
    set_pan_id(id, tx_final_msg);
    set_pan_id(id, rx_report_msg);

    return 0;
}

/**
 * @brief Sets the multiplication factor to convert carrier integrator value to
 * a frequency offset in Hertz depending on the data rate.
 *
 * @param[in] datarate_110k `true` if data rate is 110K, `false` otherwise
 */
void set_freq_offset_multiplier(bool datarate_110k) {
    if (datarate_110k) {
        freq_offset_multiplier = FREQ_OFFSET_MULTIPLIER_110KB;
    } else {
        freq_offset_multiplier = FREQ_OFFSET_MULTIPLIER;
    }
}

/**
 * @brief Sets the multiplication factor to convert frequency offset in Hertz to
 * PPM crystal offset depending on the channel.
 *
 * @param[in] channel The channel being used for UWB
 */
void set_hertz_to_ppm_multiplier(uint8_t channel) {
    switch (channel) {
    case 1:
        hertz_to_ppm_multiplier = HERTZ_TO_PPM_MULTIPLIER_CHAN_1;
        break;
    case 2:
        hertz_to_ppm_multiplier = HERTZ_TO_PPM_MULTIPLIER_CHAN_2;
        break;
    case 3:
        hertz_to_ppm_multiplier = HERTZ_TO_PPM_MULTIPLIER_CHAN_3;
        break;
    case 4:
    case 5:
    case 7:
        hertz_to_ppm_multiplier = HERTZ_TO_PPM_MULTIPLIER_CHAN_5;
        break;
    default:
        __ASSERT(false, "Invalid UWB channel (%u)", channel);
        break;
    }
}

/**
 * @brief Sets the destination address (The node being ranged to) in the
 * initiator messages.
 *
 * @param[in] id The destination address
 *
 * @note This sets the source address for the messages being received.
 */
static void set_destination(uint16_t id) {
    set_dest_id(id, tx_poll_msg);
    set_src_id(id, rx_resp_msg);
    set_dest_id(id, tx_final_msg);
    set_src_id(id, rx_report_msg);
}

#if IS_ENABLED(CONFIG_UWB_LOGIC_CLK)
/**
 * The ID associated with a ranging exchange.
 */
static uint32_t exchange_id = UINT32_C(0);

/**
 * @brief Updates each of the messages with the appropriate exchange ID.
 */
static void set_exchange_id(void) {
    SET_EXCHANGE_ID(tx_poll_msg + LOGIC_CLK_OFFSET, exchange_id);
    SET_EXCHANGE_ID(rx_resp_msg + LOGIC_CLK_OFFSET, exchange_id);
    SET_EXCHANGE_ID(tx_final_msg + LOGIC_CLK_OFFSET, exchange_id);
    SET_EXCHANGE_ID(rx_report_msg + LOGIC_CLK_OFFSET, exchange_id);
}

/**
 * Saves the exchange ID in a pointer and updates the ID for the next run.
 *
 * @param[out] x The pointer to save the exchange ID in
 */
#define update_exchange(x)                                                     \
    do {                                                                       \
        if ((x) != NULL) {                                                     \
            *(x) = exchange_id;                                                \
        }                                                                      \
        exchange_id++;                                                         \
    } while (0)
#else
/**
 * Placeholder for when the logic clock is disabled.
 */
#define set_exchange_id()  (void)0

/**
 * Placeholder for when the logic clock is disabled.
 */
#define update_exchange(x) (void)0
#endif

/**
 * @brief Sends a poll message to the node being ranged to.
 *
 * @return 0 upon success
 * @return -EBADMSG if transmission failed
 */
static int send_poll(void) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

    int check_poll_msg =
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    if (check_poll_msg != DWT_SUCCESS) {
        return -EBADMSG;
    }

    UWB_WAIT(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    return 0;
}

/**
 * @brief Waits for a response from the node being ranged to assuming the
 * double-sided ranging scheme is being used.
 *
 * @return 0 upon success
 * @return -EBADMSG if there was an rx error, rx timeout, or the message
 * received did not match the expected message
 */
static int ds_rx_response(void) {
    uint32 status_reg, frame_len;

    UWB_WAIT((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR));

    if (!(status_reg & SYS_STATUS_RXFCG)) {
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        dwt_rxreset();
        return -EBADMSG;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    rx_buffer[SEQ_CNT_OFFSET] = 0;

    if (!(memcmp(rx_buffer, rx_resp_msg, DW_BASE_LEN) == 0)) {
        return -EBADMSG;
    }

    return 0;
}

/**
 * @brief Sends the final message to the node being ranged to.
 *
 * @return 0 upon success
 * @return -ETIMEDOUT if transmission failed
 */
static int send_final(void) {
    uint64 poll_tx_ts, resp_rx_ts;
    uint64 ts_replyA_end;
    uint32 resp_tx_time;
    int ret;

    poll_tx_ts = get_tx_timestamp_u64();
    resp_rx_ts = get_rx_timestamp_u64();

    resp_tx_time =
        (resp_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    ts_replyA_end = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    msg_set_ts(&tx_final_msg[RESP_MSG_POLL_RX_TS_IDX], poll_tx_ts);
    msg_set_ts(&tx_final_msg[RESP_MSG_RESP_TX_TS_IDX], resp_rx_ts);
    msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], ts_replyA_end);

    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
    dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1);

    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if (ret != DWT_SUCCESS) {
        dwt_rxreset();
        return -ETIMEDOUT;
    }

    UWB_WAIT(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    return 0;
}

/**
 * @brief Waits for and receives the report message from the node being ranged
 * to. Additionally calculates the distance between the two nodes from the
 * payload in the report.
 *
 * @param[out] distance The estimated distance between the two nodes
 *
 * @return 0 upon success
 * @return -EBADMSG if there was an rx error, rx timeout, or the message
 * received did not match the expected message
 */
static int rx_report(double *distance) {
    uint32 status_reg, frame_len;
    uint32_t msg_tof_dtu;
    double tof;

    UWB_WAIT((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR));

    if (!(status_reg & SYS_STATUS_RXFCG)) {
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        dwt_rxreset();
        return -EBADMSG;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
    if (frame_len <= RX_BUFFER_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    rx_buffer[SEQ_CNT_OFFSET] = 0;

    if (!(memcmp(rx_buffer, rx_report_msg, DW_BASE_LEN) == 0)) {
        return -EBADMSG;
    }

    msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &msg_tof_dtu);
    tof = msg_tof_dtu * DWT_TIME_UNITS;
    *distance = tof * SPEED_OF_LIGHT;
    return 0;
}

/**
 * @brief Initiates a double-sided two-way ranging measurement to a certain
 * node.
 *
 * @param[in] id The node to range to
 * @param[out] distance The estimated distance between the nodes
 * @param[out] logic_clock The ID associated with the successful exchange.
 *
 * @return 0 upon a successful ranging run
 * @return -EINVAL if distance parameter is NULL
 * @return negative error code otherwise
 *
 * @note logic_clock will be updated after a successful double-sided two-way
 * ranging exchange. If an error occurred, then logic_clock will not be updated
 * and thus will retain its original value. Also, if logic_clock is NULL, it is
 * assumed that the logic_clock output is not desired and the run will still be
 * initiated.
 */
int ds_init_run(uint16_t id, double *distance, uint32_t *logic_clock) {
    int err;

    if (distance == NULL) {
        return -EINVAL;
    }

    set_destination(id);
    set_exchange_id();

    if ((err = send_poll()) < 0) {
        return err;
    }

    if ((err = ds_rx_response()) < 0) {
        return err;
    }

    if ((err = send_final()) < 0) {
        return err;
    }

    if ((err = rx_report(distance)) < 0) {
        return err;
    }

    update_exchange(logic_clock);

    return 0;
}

/**
 * @brief Waits for a response from the node being ranged to assuming the
 * single-sided two-way ranging scheme is being used. Additionally calculates
 * the estimated distance between the nodes.
 *
 * @param[out] distance The estimated distance between the nodes
 *
 * @return 0 upon success
 * @return -EBADMSG if there was an rx error, rx timeout, or the message
 * received did not match the expected message
 */
static int ss_rx_response(double *distance) {
    uint32_t status_reg, frame_len, poll_tx_ts, resp_rx_ts, poll_rx_ts,
        resp_tx_ts;
    int32_t rtd_init, rtd_resp;
    float clockOffsetRatio;
    double tof;

    UWB_WAIT((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR));

    if (!(status_reg & SYS_STATUS_RXFCG)) {
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        dwt_rxreset();
        return -EBADMSG;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    rx_buffer[SEQ_CNT_OFFSET] = 0;

    if (!(memcmp(rx_buffer, rx_resp_msg, DW_BASE_LEN) == 0)) {
        dwt_rxreset();
        return -EBADMSG;
    }

    poll_tx_ts = dwt_readtxtimestamplo32();
    resp_rx_ts = dwt_readrxtimestamplo32();

    clockOffsetRatio =
        (float)(dwt_readcarrierintegrator() *
                (freq_offset_multiplier * hertz_to_ppm_multiplier / 1.0e6));

    msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
    msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

    rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
    rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);

    tof = (double)(((float)rtd_init -
                    (float)rtd_resp * (1.0f - clockOffsetRatio)) /
                   2.0f) *
          DWT_TIME_UNITS;
    *distance = tof * SPEED_OF_LIGHT;
    return 0;
}

/**
 * @brief Initiates a single-sided two-way ranging measurement to a certain
 * node.
 *
 * @param[in] id The node to range to
 * @param[out] distance The estimated distance between the nodes
 * @param[out] logic_clock The ID associated with the successful exchange.
 *
 * @return 0 upon a successful ranging run
 * @return -EINVAL if distance parameter is NULL
 * @return negative error code otherwise
 *
 * @note logic_clock will be updated after a successful single-sided two-way
 * ranging exchange. If an error occurred, then logic_clock will not be updated
 * and thus will retain its original value. Also, if logic_clock is NULL, it is
 * assumed that the logic_clock output is not desired and the run will still be
 * initiated.
 */
int ss_init_run(uint16_t id, double *distance, uint32_t *logic_clock) {
    int err;

    if (distance == NULL) {
        return -EINVAL;
    }

    set_destination(id);
    set_exchange_id();

    if ((err = send_poll()) < 0) {
        return err;
    }

    if ((err = ss_rx_response(distance)) < 0) {
        return err;
    }

    update_exchange(logic_clock);

    return 0;
}
