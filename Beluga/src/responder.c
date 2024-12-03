/*! ----------------------------------------------------------------------------
 *  @file    resp_main.c
 *  @brief   Double-sided and Single-sided two-way ranging (DS/SS TWR) responder
 * code
 *
 *
 *           Notes at the end of this file, to expand on the inline comments.
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include <ble_app.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <init_resp_common.h>
#include <port_platform.h>
#include <responder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app_leds.h>

LOG_MODULE_REGISTER(responder_logger, LOG_LEVEL_INF);

K_SEM_DEFINE(k_sus_resp, 0, 1);

/* Frames used in the ranging process. See NOTE 2,3 below. */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0,   0xCA, 0xDE, 'W',
                              'A',  0,   0, 0x61, 0,    0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0,    0xCA, 0xDE, 'V', 'E',
                              'W',  'A',  0x50, 0,    0,    0,   0,
                              0,    0,    0,    0,    0,    0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V',
                               'E',  0x69, 0, 0,    0,    0,   0,   0,
                               0,    0,    0, 0,    0,    0,   0,   0};
static uint8 tx_report_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W',
                                'A',  0xE3, 0, 0,    0,    0,   0,   0};

#define RX_BUF_LEN MAX(POLL_MSG_LEN, FINAL_MSG_LEN)
static uint8 rx_buffer[RX_BUF_LEN];

// Not enough time to write the data so TX timeout extended for nRF operation.
// Might be able to get away with 800 uSec but would have to test
// See note 6 at the end of this file
#define POLL_RX_TO_RESP_TX_DLY_UUS 1500

int set_responder_id(uint16_t id) {
    CHECK_UWB_ACTIVE();

    set_dest_id(id, rx_poll_msg);
    set_src_id(id, tx_resp_msg);
    set_dest_id(id, rx_final_msg);
    set_src_id(id, tx_report_msg);

    return 0;
}

static int wait_poll_message(uint16_t *src_id) {
    uint32 status_reg, frame_len;

    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    UWB_WAIT(
        (status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
        (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {

        if (k_sem_count_get(&k_sus_resp) == 0) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            LOG_INF("Responder suspended");
            return -EBUSY;
        }
    }

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
    *src_id = get_src_id(rx_buffer);
    rx_buffer[SRC_OFFSET] = 0;
    rx_buffer[SRC_OFFSET+1] = 0;
    if (!(memcmp(rx_buffer, rx_poll_msg, DW_BASE_LEN) == 0)) {
        return -EBADMSG;
    }

    return 0;
}

static int ds_respond(uint64_t *poll_rx_ts) {
    uint32 resp_tx_time;
    int ret;

    *poll_rx_ts = get_rx_timestamp_u64();

    resp_tx_time =
        (*poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if (ret != DWT_SUCCESS) {
        dwt_rxreset();
        return -EBADMSG;
    }

    UWB_WAIT(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    return 0;
}

static int wait_final(uint64 *tof_dtu,
                      const uint64_t *poll_rx_ts) {
    uint32 status_reg, frame_len, poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
    uint32_t resp_rx_ts, poll_tx_ts, final_tx_ts;
    uint64_t final_rx_ts, resp_tx_ts;
    double roundA, replyA, roundB, replyB;

    UWB_WAIT(
        (status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
        (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
        if (k_sem_count_get(&k_sus_resp) == 0) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            LOG_INF("Responder got suspended");
            return -EBUSY;
        }
    }

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

    if (!(memcmp(rx_buffer, rx_final_msg, DW_BASE_LEN) == 0)) {
        return -EBADMSG;
    }

    final_rx_ts = get_rx_timestamp_u64();
    resp_tx_ts = get_tx_timestamp_u64();

    msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_tx_ts);
    msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_rx_ts);
    msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

    poll_rx_ts_32 = (uint32)(*poll_rx_ts);
    resp_tx_ts_32 = (uint32)resp_tx_ts;
    final_rx_ts_32 = (uint32)final_rx_ts;
    roundB = (double)(final_rx_ts_32 - resp_tx_ts_32);
    replyB = (double)(resp_tx_ts_32 - poll_rx_ts_32);
    roundA = (double)(resp_rx_ts - poll_tx_ts);
    replyA = (double)(final_tx_ts - resp_rx_ts);

    if ((roundA * roundB - replyA * replyB) <= 0) {
        LOG_INF("Bad TOF response");
        return -EINVAL;
    }

    *tof_dtu = (uint64)((roundA * roundB - replyA * replyB) /
                        (roundA + roundB + replyA + replyB));

    return 0;
}

static int send_report(uint64 tof_dtu) {
    msg_set_ts(&tx_report_msg[RESP_MSG_POLL_RX_TS_IDX], tof_dtu);

    dwt_writetxdata(sizeof(tx_report_msg), tx_report_msg, 0);
    dwt_writetxfctrl(sizeof(tx_report_msg), 0, 1);
    int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

    if (ret != DWT_SUCCESS) {
        dwt_rxreset();
        LOG_INF("Failed to transmit");
        return -EBADMSG;
    }

    UWB_WAIT(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    return 0;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn ss_resp_run()
 *
 * @brief perform double-sided responses to signal initiator
 *
 * @param  none
 *
 * @return int represent task complete or abort
 */
int ds_resp_run(void) {
    int err;
    uint16_t src_id;
    uint64 tof_dtu;
    uint64_t poll_rx_ts;

    if (k_sem_count_get(&k_sus_resp) == 0) {
        return -EBUSY;
    }

    if ((err = wait_poll_message(&src_id)) < 0) {
        return err;
    }

    set_dest_id(src_id, tx_resp_msg);

    if ((err = ds_respond(&poll_rx_ts)) < 0) {
        return err;
    }

    set_src_id(src_id, rx_final_msg);

    if ((err = wait_final(&tof_dtu, &poll_rx_ts)) < 0) {
        return err;
    }

    set_dest_id(src_id, tx_report_msg);

    if ((err = send_report(tof_dtu)) < 0) {
        return err;
    }

    return 0;
}

static int ss_respond(void) {
    uint32 resp_tx_time;
    int ret;
    uint64_t poll_rx_ts, resp_tx_ts;

    poll_rx_ts = get_rx_timestamp_u64();

    resp_tx_time =
        (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
    msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

    ret = dwt_starttx(DWT_START_TX_DELAYED);

    if (ret != DWT_SUCCESS) {
        dwt_rxreset();
        return -EBADMSG;
    }

    UWB_WAIT(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS) {
        if (0 == k_sem_count_get(&k_sus_resp)) {
            dwt_forcetrxoff();
            return -EBUSY;
        }
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    return 0;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn ss_resp_run()
 *
 * @brief perform single-sided responses to signal initiator
 *
 * @param  none
 *
 * @return int represent task complete or abort
 */
int ss_resp_run(void) {
    int err;
    uint16_t src_id;
    if (k_sem_count_get(&k_sus_resp) == 0) {
        return -EBUSY;
    }

    if ((err = wait_poll_message(&src_id)) < 0) {
        dwt_rxreset();
        return err;
    }

    set_dest_id(src_id, tx_resp_msg);

    if ((err = ss_respond()) < 0) {
        return err;
    }

    return 0;
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. This is the task delay when using FreeRTOS. Task is delayed a given number
 *of ticks. Useful to be able to define this out to see the effect of the RTOS
 *    on timing.
 * 2. The frames used here are Decawave specific ranging frames, complying with
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
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is
 *in the ranging process). The remaining bytes are specific to each message as
 *follows: Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10 -> 13: poll message reception timestamp.
 *     - byte 14 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example
 *to keep it simple but for a real product every device should have a unique ID.
 *Here, 16-bit addressing is used to keep the messages as short as possible but,
 *in an actual application, this should be done only after an exchange of
 *specific messages used to define those short addresses for each device
 *participating to the ranging exchange.
 * 4. dwt_writetxdata() takes the full size of the message as a parameter but
 *only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could
 *be two bytes shorter without losing any data (but the sizeof would not work
 *anymore then as we would still have to indicate the full length of the frame
 *to dwt_writetxdata()).
 * 5. We use polled mode of operation here to keep the example as simple as
 *possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also
 *to be noted that STATUS register is 5 bytes long but, as the event we use are
 *all in the first bytes of the register, we can use the simple
 *dwt_read32bitreg() API call to access it instead of reading the whole 5 bytes.
 * 6. POLL_RX_TO_RESP_TX_DLY_UUS is a critical value for porting to different
 *processors. For slower platforms where the SPI is at a slower speed or the
 *processor is operating at a lower frequency (Comparing to STM32F, SPI of 18MHz
 *and Processor internal 72MHz)this value needs to be increased. Knowing the
 *exact time when the responder is going to send its response is vital for time
 *of flight calculation. The specification of the time of respnse must allow the
 *processor enough time to do its calculations and put the packet in the Tx
 *buffer. So more time required for a slower system(processor).
 * 7. As we want to send final TX timestamp in the final message, we have to
 *compute it in advance instead of relying on the reading of DW1000 register.
 *Timestamps and delayed transmission time are both expressed in device time
 *units so we just have to add the desired response delay to response RX
 *timestamp to get final transmission time. The delayed transmission time
 *resolution is 512 device time units which means that the lower 9 bits of the
 *obtained value must be zeroed. This also allows to encode the 40-bit value in
 *a 32-bit words by shifting the all-zero lower 8 bits.
 * 8. In this operation, the high order byte of each 40-bit timestamps is
 *discarded. This is acceptable as those time-stamps are not separated by more
 *than 2**32 device time units (which is around 67 ms) which means that the
 *calculation of the round-trip delays (needed in the time-of-flight
 *computation) can be handled by a 32-bit subtraction.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but
 *only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could
 *be two bytes shorter without losing any data (but the sizeof would not work
 *anymore then as we would still have to indicate the full length of the frame
 *to dwt_writetxdata()).
 *10. The user is referred to DecaRanging ARM application (distributed with
 *EVK1000 product) for additional practical example of usage, and to the DW1000
 *API Guide for more details on the DW1000 driver functions.
 *
 ****************************************************************************************************************************************************/
