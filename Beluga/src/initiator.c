/*! ----------------------------------------------------------------------------
 *  @file    init_main.c
 *  @brief   Double-sided and Single-sided two-way ranging (DS/SS TWR) initiator
 * code
 *
 *
 *           Notes at the end of this file, expand on the inline comments.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include <deca_device_api.h>
#include <deca_regs.h>
#include <init_resp_common.h>
#include <initiator.h>
#include <port_platform.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app_leds.h>

LOG_MODULE_REGISTER(initializer_logger, LOG_LEVEL_INF);

K_SEM_DEFINE(k_sus_init, 0, 1);

/* Frames used in the ranging process. See NOTE 1,2 below. */
static uint8 tx_poll_msg[POLL_MSG_LEN] = {0x41, 0x88, 0,   0xCA, 0xDE, 'W',
                              'A',  'V',  'E', 0x61, 0,    0};
static uint8 rx_resp_msg[RESP_MSG_LEN] = {0x41, 0x88, 0,    0xCA, 0xDE, 'V', 'E',
                              'W',  'A',  0x50, 0,    0,    0,   0,
                              0,    0,    0,    0,    0,    0};
static uint8 tx_final_msg[FINAL_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V',
                               'E',  0x69, 0, 0,    0,    0,   0,   0,
                               0,    0,    0, 0,    0,    0,   0,   0};
static uint8 rx_report_msg[REPORT_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W',
                                'A',  0xE3, 0, 0,    0,    0,   0,   0};

#define RX_BUF_LEN MAX(RESP_MSG_LEN, REPORT_MSG_LEN)
static uint8 rx_buffer[RX_BUF_LEN];

#define POLL_RX_TO_RESP_TX_DLY_UUS 2000

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 300

int set_initiator_id(uint16_t id) {
    CHECK_UWB_ACTIVE();

    set_src_id(id, tx_poll_msg);
    set_dest_id(id, rx_resp_msg);
    set_src_id(id, tx_final_msg);
    set_dest_id(id, rx_report_msg);

    return 0;
}

static void set_destination(uint16_t id) {
    set_dest_id(id, tx_poll_msg);
    set_src_id(id, rx_resp_msg);
    set_dest_id(id, tx_final_msg);
    set_src_id(id, rx_report_msg);
}

#if IS_ENABLED(CONFIG_UWB_LOGIC_CLK)
static uint32_t exchange_id = UINT32_C(0);

static void set_exchange_id(void) {
    SET_EXCHANGE_ID(tx_poll_msg + LOGIC_CLK_OFFSET, exchange_id);
    SET_EXCHANGE_ID(rx_resp_msg + LOGIC_CLK_OFFSET, exchange_id);
    SET_EXCHANGE_ID(tx_final_msg + LOGIC_CLK_OFFSET, exchange_id);
    SET_EXCHANGE_ID(rx_report_msg + LOGIC_CLK_OFFSET, exchange_id);
}

#define update_exchange(x) x = exchange_id++
#else
#define set_exchange_id() (void)0
#define update_exchange(x) ARG_UNUSED(x)
#endif

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

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn ds_init_run()
 *
 * @brief Initiate UWB double-sided two way ranging
 *
 * @param  node ID
 *
 * @return distance between sending nodes and id node
 */
int ds_init_run(uint16_t id, double *distance, uint32_t *logic_clock) {
    int err;

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

    update_exchange(*logic_clock);

    return 0;
}

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

    clockOffsetRatio = (float)(dwt_readcarrierintegrator() *
                               (FREQ_OFFSET_MULTIPLIER *
                                HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6));

    msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
    msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

    rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
    rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);

    tof = (((float)rtd_init - (float)rtd_resp * (1.0f - clockOffsetRatio)) /
           2.0f) *
          DWT_TIME_UNITS;
    *distance = tof * SPEED_OF_LIGHT;
    return 0;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn ss_init_run()
 *
 * @brief Initiate UWB single-sided two way ranging
 *
 * @param  node ID
 *
 * @return distance between sending nodes and id node
 */
int ss_init_run(uint16_t id, double *distance, uint32_t *logic_clock) {
    int err;

    set_destination(id);
    set_exchange_id();

    if ((err = send_poll()) < 0) {
        return err;
    }

    if ((err = ss_rx_response(distance)) < 0) {
        return err;
    }

    update_exchange(*logic_clock);

    return 0;
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The frames used here are Decawave specific ranging frames, complying with
 *the IEEE 802.15.4 standard data frame encoding. The frames are the following:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder to complete the exchange and
 *provide all information needed by the initiator to compute the time-of-flight
 *(distance) estimate. The first 10 bytes of those frame are common and are
 *composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit
 *addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 2 below.
 *     - byte 7/8: source address, see NOTE 2 below.
 *     - byte 9: function code (specific values to indicate which message it is
 *in the ranging process). The remaining bytes are specific to each message as
 *follows: Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10 -> 13: poll message reception timestamp.
 *     - byte 14 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 2. Source and destination addresses are hard coded constants in this example
 *to keep it simple but for a real product every device should have a unique ID.
 *Here, 16-bit addressing is used to keep the messages as short as possible but,
 *in an actual application, this should be done only after an exchange of
 *specific messages used to define those short addresses for each device
 *participating to the ranging exchange.
 * 3. dwt_writetxdata() takes the full size of the message as a parameter but
 *only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could
 *be two bytes shorter without losing any data (but the sizeof would not work
 *anymore then as we would still have to indicate the full length of the frame
 *to dwt_writetxdata()).
 * 4. The high order byte of each 40-bit time-stamps is discarded here. This is
 *acceptable as, on each device, those time-stamps are not separated by more
 *than 2**32 device time units (which is around 67 ms) which means that the
 *calculation of the round-trip delays can be handled by a 32-bit subtraction.
 * 5. The user is referred to DecaRanging ARM application (distributed with
 *EVK1000 product) for additional practical example of usage, and to the DW1000
 *API Guide for more details on the DW1000 driver functions.
 * 6. The use of the carrier integrator value to correct the TOF calculation,
 *was added Feb 2017 for v1.3 of this example.  This significantly improves the
 *result of the SS-TWR where the remote responder unit's clock is a number of
 *PPM offset from the local inmitiator unit's clock. As stated in NOTE 2 a fixed
 *offset in range will be seen unless the antenna delsy is calibratred and set
 *correctly.
 *
 ****************************************************************************************************************************************************/
