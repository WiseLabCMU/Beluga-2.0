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

#include "deca_device_api.h"
#include "deca_regs.h"
#include "initiator.h"
#include "port_platform.h"
#include "random.h"
#include <app_leds.h>
#include <ble_app.h>
#include <init_resp_common.h>
#include <math.h>
#include <responder.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(responder_logger, LOG_LEVEL_INF);

K_SEM_DEFINE(k_sus_resp, 0, 1);

#define RX_BUF_LEN MAX(POLL_MSG_LEN, FINAL_MSG_LEN)

static uint8 rx_poll_msg[POLL_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W',
                                          'A',  0,    0, 0x61, 0,    0}; // 12
static uint8 tx_resp_msg[RESP_MSG_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 'W', 'A', 0x50,
    0x02, 0,    0, 0,    0,    0, 0, 0,   0,   0}; // 20
static uint8 rx_final_msg[FINAL_MSG_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 0, 0, 0x69, 0, 0,
    0,    0,    0, 0,    0,    0,   0,   0, 0, 0,    0, 0}; // 24
static uint8 tx_report_msg[REPORT_MSG_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 'W', 'A', 0xE3, 0, 0, 0, 0, 0, 0}; // 16

uint8 sequence_count = 0;
static uint8 rx_buffer[RX_BUF_LEN];

int set_responder_pan_id(uint16_t id) {
    CHECK_UWB_ACTIVE();

    memcpy(rx_poll_msg + PAN_ID_OFFSET, &id, PAN_ID_OVERHEAD);
    memcpy(tx_resp_msg + PAN_ID_OFFSET, &id, PAN_ID_OVERHEAD);
    memcpy(rx_final_msg + PAN_ID_OFFSET, &id, PAN_ID_OVERHEAD);
    memcpy(tx_report_msg + PAN_ID_OFFSET, &id, PAN_ID_OVERHEAD);

    return 0;
}

int set_responder_id(uint16_t id) {
    CHECK_UWB_ACTIVE();

    memcpy(rx_poll_msg + DEST_OFFSET, &id, DEST_OVERHEAD);
    memcpy(tx_resp_msg + SRC_OFFSET, &id, SRC_OVERHEAD);
    memcpy(rx_final_msg + DEST_OFFSET, &id, DEST_OVERHEAD);
    memcpy(tx_report_msg + SRC_OFFSET, &id, SRC_OVERHEAD);

    return 0;
}

ALWAYS_INLINE static int wait_for_poll_message(void) {
    int status;
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    UWB_WAIT(
        (status = dwt_read32bitreg(SYS_STATUS_ID)) &
        (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
        if (k_sem_count_get(&k_sus_resp) == 0) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            LOG_INF("Responder suspended by initiator");
            return -EBUSY;
        }
    }

    if (!(status & SYS_STATUS_RXFCG)) {
        dwt_write32bitreg(SYS_STATUS_ID,
                          (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR));
        dwt_rxreset();
        LOG_WRN("Receive error detected");
        return -ETIME;
    }

    return 0;
}

ALWAYS_INLINE static int receive_poll_message(uint16_t *poll_src) {
    uint32_t frame_len;
    uint8 seq_cnt;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    memcpy(poll_src, rx_buffer + SRC_OFFSET, sizeof(*poll_src));
    memset(rx_buffer + SRC_OFFSET, 0, sizeof(*poll_src));
    seq_cnt = rx_buffer[SEQ_CNT_OFFSET];

    rx_buffer[SEQ_CNT_OFFSET] = 0;

    if (memcmp(rx_buffer, rx_poll_msg, DW_BASE_LEN) != 0) {
        // Not meant for this node
        return -EINVAL;
    }

    return 0;
}

ALWAYS_INLINE static int double_sided_respond(uint16_t id,
                                              uint64_t *poll_rx_ts) {
    uint32_t resp_tx_time;

    *poll_rx_ts = get_rx_timestamp_u64();

    resp_tx_time =
        (*poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    memcpy(tx_resp_msg + DEST_OFFSET, &id, DEST_OVERHEAD);

    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

    tx_resp_msg[SEQ_CNT_OFFSET] = sequence_count;
    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
    if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) !=
        DWT_SUCCESS) {
        dwt_rxreset();
        LOG_WRN("Unable to start response transmission");
        return -ETIME;
    }

    return 0;
}

ALWAYS_INLINE static int wait_for_final(void) {
    int status;

    UWB_WAIT(
        (status = dwt_read32bitreg(SYS_STATUS_ID)) &
        (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
        if (k_sem_count_get(&k_sus_resp) == 0) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            LOG_INF("Responder suspended by initiator");
            return -EBUSY;
        }
    }

    sequence_count++;

    if (!(status & SYS_STATUS_RXFCG)) {
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        dwt_rxreset();
        return -ETIME;
    }

    return 0;
}

ALWAYS_INLINE static int receive_final(uint16_t src_id) {
    uint32_t frame_len;
    uint8 seq_count;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;

    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    memcpy(rx_final_msg + SRC_OFFSET, &src_id, sizeof(src_id));
    seq_count = rx_buffer[SEQ_CNT_OFFSET];
    rx_buffer[SEQ_CNT_OFFSET] = 0;

    if (memcmp(rx_buffer, rx_final_msg, DW_BASE_LEN) != 0) {
        return -EINVAL;
    }

    return 0;
}

ALWAYS_INLINE static int process_final_message(uint64_t poll_rx_ts,
                                               int64_t *tof_dtu) {
    uint64_t resp_tx_ts, final_rx_ts;
    uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
    uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
    double Ra, Rb, Da, Db, tof_dtu_num, tof;

    resp_tx_ts = get_tx_timestamp_u64();
    final_rx_ts = get_rx_timestamp_u64();

    memcpy(&poll_tx_ts, rx_buffer + FINAL_TX_POLL_TS_OFFSET,
           TIMESTAMP_OVERHEAD);
    memcpy(&resp_rx_ts, rx_buffer + FINAL_RX_RESP_TS_OFFSET,
           TIMESTAMP_OVERHEAD);
    memcpy(&final_tx_ts, rx_buffer + FINAL_TX_FINAL_TS_OFFSET,
           TIMESTAMP_OVERHEAD);

    poll_rx_ts_32 = (uint32)poll_rx_ts;
    resp_tx_ts_32 = (uint32)resp_tx_ts;
    final_rx_ts_32 = (uint32)final_rx_ts;

    Ra = (double)(resp_rx_ts - poll_tx_ts);
    Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
    Da = (double)(final_tx_ts - resp_rx_ts);
    Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
    tof_dtu_num = (Ra * Rb) - (Da * Db);

    if (islessequal(tof_dtu_num, 0.0)) {
        LOG_WRN("Bad TOF numerator");
        return -EINVAL;
    }

    *tof_dtu = (int64_t)(tof_dtu_num / (Ra + Rb + Da + Db));

    return 0;
}

ALWAYS_INLINE static int send_report(uint16_t src_id, int64_t tof_dtu) {
    int ret;

    memcpy(tx_report_msg + DEST_OFFSET, &src_id, DEST_OVERHEAD);
    memcpy(tx_report_msg + REPORT_TOF_OFFSET, &tof_dtu, TIMESTAMP_OVERHEAD);

    dwt_writetxdata(sizeof(tx_report_msg), tx_report_msg, 0);
    dwt_writetxfctrl(sizeof(tx_report_msg), 0, 1);

    ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

    if (ret != DWT_SUCCESS) {
        return -EAGAIN;
    }

    UWB_WAIT(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS);

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    return 0;
}

int ds_respond(void) {
    int err;
    uint16_t src_id;
    uint64_t poll_rx_ts;
    int64_t tof_dtu;

    if (k_sem_count_get(&k_sus_resp) == 0) {
        return -EBUSY;
    }

    if ((err = wait_for_poll_message()) < 0) {
        return err;
    }

    if ((err = receive_poll_message(&src_id)) < 0) {
        return err;
    }

    if ((err = double_sided_respond(src_id, &poll_rx_ts)) < 0) {
        return err;
    }

    if ((err = wait_for_final()) < 0) {
        return err;
    }

    if ((err = receive_final(src_id)) < 0) {
        return err;
    }

    if ((err = process_final_message(poll_rx_ts, &tof_dtu)) < 0) {
        return err;
    }

    if ((err = send_report(src_id, tof_dtu)) < 0) {
        return err;
    }

    return 0;
}

ALWAYS_INLINE static int single_sided_respond(uint16_t src) {
    uint32 resp_tx_time;
    uint64_t poll_rx_ts, resp_tx_ts;

    poll_rx_ts = get_rx_timestamp_u64();

    resp_tx_time =
        (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

    dwt_setdelayedtrxtime(resp_tx_time);

    resp_tx_ts =
        (((uint64_t)(resp_tx_time & UINT64_C(0xFFFFFFFE))) << 8) + TX_ANT_DLY;

    memcpy(tx_resp_msg + SS_RESP_RX_POLL_TS_OFFSET, &poll_rx_ts,
           TIMESTAMP_OVERHEAD);
    memcpy(tx_resp_msg + SS_RESP_TX_RESP_TS_OFFSET, &resp_tx_ts,
           TIMESTAMP_OVERHEAD);
    memcpy(tx_resp_msg + DEST_OFFSET, &src, DEST_OVERHEAD);

    tx_resp_msg[SEQ_CNT_OFFSET] = sequence_count;

    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

    if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
        LOG_WRN("Unable to start delayed response");
        dwt_rxreset();
        return -ETIME;
    }

    UWB_WAIT(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS) {
        unsigned int suspend = k_sem_count_get(&k_sus_resp);
        if (suspend == 0) {
            dwt_forcetrxoff();
            return EBUSY;
        }
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    sequence_count++;

    return 0;
}

int ss_respond(void) {
    int err;
    uint16_t src_id;

    if (k_sem_count_get(&k_sus_resp) == 0) {
        return -EBUSY;
    }

    if ((err = wait_for_poll_message()) < 0) {
        return err;
    }

    if ((err = receive_poll_message(&src_id)) < 0) {
        return err;
    }

    if ((err = single_sided_respond(src_id)) < 0) {
        return err;
    }

    return 0;
}
