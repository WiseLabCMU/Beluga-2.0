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
#include <ble_app.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app_leds.h>
#include <math.h>

LOG_MODULE_REGISTER(responder_logger, LOG_LEVEL_INF);

K_SEM_DEFINE(k_sus_resp, 0, 1);

// TODO: Build config for this
#define DW_LOGIC_CLOCK_OVERHEAD 0

#define DW_BASE_LENGTH 10
#define DW_TIME_OVERHEAD 4
#define DW_CRC_OVERHEAD 2

#define POLL_MSG_LEN (DW_BASE_LENGTH + DW_LOGIC_CLOCK_OVERHEAD + DW_CRC_OVERHEAD)
#define RESP_MSG_LEN (DW_BASE_LENGTH + DW_LOGIC_CLOCK_OVERHEAD + DW_TIME_OVERHEAD + )

// TODO: Come back to this
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 0, 0, 0x21, 0, 0}; // 12
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 'W', 'A', 0x10, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 20
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 0, 0, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 24
static uint8 tx_report_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 'W', 'A', 0xE3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 20

#define SEQUENCE_COUNTER_OFFSET 2
#define SEQUENCE_COUNTER_BYTES  sizeof(uint8_t)

#define PAN_ID_OFFSET           3
#define PAN_ID_BYTES            sizeof(uint16_t)

#define DESTINATION_OFFSET      5
#define DESTINATION_BYTES       sizeof(uint16_t)

#define SOURCE_OFFSET           7
#define SOURCE_BYTES            sizeof(uint16_t)

uint8 sequence_count = 0;

// TODO: Come back to this
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

#define CHECK_UWB_ACTIVE()                                                     \
    do {                                                                       \
        if (get_uwb_led_state() == LED_UWB_ON) {                               \
            return -EBUSY;                                                     \
        }                                                                      \
    } while (0)

#define UWB_WAIT(cond) while (!(cond))

int set_responder_pan_id(uint16_t id) {
    CHECK_UWB_ACTIVE();

    memcpy(rx_poll_msg + PAN_ID_OFFSET, &id, PAN_ID_BYTES);
    memcpy(tx_resp_msg + PAN_ID_OFFSET, &id, PAN_ID_BYTES);
    memcpy(rx_final_msg + PAN_ID_OFFSET, &id, PAN_ID_BYTES);
    memcpy(tx_report_msg + PAN_ID_OFFSET, &id, PAN_ID_BYTES);

    return 0;
}

int set_responder_id(uint16_t id) {
    CHECK_UWB_ACTIVE();

    memcpy(rx_poll_msg + DESTINATION_OFFSET, &id, DESTINATION_BYTES);
    memcpy(tx_resp_msg + SOURCE_OFFSET, &id, SOURCE_BYTES);
    memcpy(rx_final_msg + DESTINATION_OFFSET, &id, DESTINATION_BYTES);
    memcpy(tx_report_msg + SOURCE_OFFSET, &id, SOURCE_BYTES);

    return 0;
}

ALWAYS_INLINE static int wait_for_poll_message(void) {
    int status;
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    UWB_WAIT((status = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
        if (k_sem_count_get(&k_sus_resp) == 0) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            LOG_INF("Responder suspended by initiator");
            return -EBUSY;
        }
    }

    if (!(status & SYS_STATUS_RXFCG)) {
        dwt_write32bitreg(SYS_STATUS_ID, (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR));
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

    memcpy(poll_src, rx_buffer + SOURCE_OFFSET, sizeof(*poll_src));
    memset(rx_buffer + SOURCE_OFFSET, 0, sizeof(*poll_src));
    seq_cnt = rx_buffer[SEQUENCE_COUNTER_OFFSET];

    rx_buffer[SEQUENCE_COUNTER_OFFSET] = 0;

    if (memcmp(rx_buffer, rx_poll_msg, DW_BASE_LENGTH) != 0) {
        // Not meant for this node
        return -EINVAL;
    }

    return 0;
}

#define UUS_TO_DWT_TIME 65536
#define POLL_RX_TO_RESP_TX_DLY_UUS 2750
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
#define FINAL_RX_TIMEOUT_UUS 3300


#define GET_UWB_TIMESTAMP(type, stamp)                                         \
    do {                                                                       \
        uint8 ts_tab[5];                                                       \
        (stamp) = 0;                                                           \
        dwt_read##type##timestamp(ts_tab);                                     \
        for (int i = 4; i >= 0; i--) {                                         \
            (stamp) <<= 8;                                                     \
            (stamp) |= ts_tab[i];                                              \
        }                                                                      \
    } while (0)

static uint64_t get_rx_timestamp_u64(void) {
    uint64_t ts;
    GET_UWB_TIMESTAMP(rx, ts);
    return ts;
}

ALWAYS_INLINE static int double_sided_respond(uint16_t id, uint64_t * poll_rx_ts) {
    uint32_t resp_tx_time;
    int ret;

    *poll_rx_ts = get_rx_timestamp_u64();

    resp_tx_time = (*poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    memcpy(tx_resp_msg + DESTINATION_OFFSET, &id, DESTINATION_BYTES);

    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

    tx_resp_msg[SEQUENCE_COUNTER_OFFSET] = sequence_count;
    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
    if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        dwt_rxreset();
        LOG_WRN("Unable to start response transmission");
        return -ETIME;
    }

    return 0;
}

ALWAYS_INLINE int wait_for_final(void) {
    int status;

    UWB_WAIT((status = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
        if (k_sem_count_get(&k_sus_resp) == 0) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            LOG_INF("Responder suspended by initiator");
            return -EBUSY;
        }
    }

    sequence_count++;

    if (!(status & SYS_STATUS_RXFCG)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        dwt_rxreset();
        return -ETIME;
    }

    return 0;
}

ALWAYS_INLINE int receive_final(uint16_t src_id) {
    uint32_t frame_len;
    uint8 seq_count;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;

    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    memcpy(rx_final_msg + SOURCE_OFFSET, &src_id, sizeof(src_id));
    seq_count = rx_buffer[SEQUENCE_COUNTER_OFFSET];
    rx_buffer[SEQUENCE_COUNTER_OFFSET] = 0;

    if (memcmp(rx_buffer, rx_final_msg, DW_BASE_LENGTH) != 0) {
        return -EINVAL;
    }

    return 0;
}

static uint64_t get_tx_timestamp_u64(void) {
    uint64_t ts;
    GET_UWB_TIMESTAMP(tx, ts);
    return ts;
}

#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18

#define GET_UWB_MSG_TIMESTAMP(ts, ts_buf, offset)                              \
    do {                                                                       \
        (ts) = 0;                                                              \
        for (int i = offset; i < 4; i++) {                                     \
            (ts) += (ts_buf)[i] << ((i - (offset)) * 8);                       \
        }                                                                      \
    } while (0)

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

ALWAYS_INLINE int process_final_message(uint64_t poll_rx_ts, int64_t *tof_dtu) {
    uint64_t resp_tx_ts, final_rx_ts;
    uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
    uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
    double Ra, Rb, Da, Db, tof_dtu_num, tof;

    resp_tx_ts = get_tx_timestamp_u64();
    final_rx_ts = get_rx_timestamp_u64();

    GET_UWB_MSG_TIMESTAMP(poll_tx_ts, rx_buffer, FINAL_MSG_POLL_TX_TS_IDX);
    GET_UWB_MSG_TIMESTAMP(resp_rx_ts, rx_buffer, FINAL_MSG_RESP_RX_TS_IDX);
    GET_UWB_MSG_TIMESTAMP(final_tx_ts, rx_buffer, FINAL_MSG_FINAL_TX_TS_IDX);

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

ALWAYS_INLINE int send_report(uint16_t src_id, int64_t tof_dtu) {
    int ret;

    memcpy(tx_report_msg + 4, &tof_dtu, 4);
    memcpy(tx_report_msg + DESTINATION_OFFSET, &src_id, DESTINATION_BYTES);

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

int ss_respond(void) {
    int err;

    if (k_sem_count_get(&k_sus_resp) == 0) {
        return -EBUSY;
    }

    if ((err = wait_for_poll_message()) < 0) {
        return err;
    }

    return 0;
}

