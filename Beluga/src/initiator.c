

#include "initiator.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "random.h"
#include <app_leds.h>
#include <init_resp_common.h>
#include <string.h>
#include <utils.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(initializer_logger, LOG_LEVEL_INF);

K_SEM_DEFINE(k_sus_init, 0, 1);

#define POLL_RX_TO_RESP_TX_DLY_UUS 2000

#define RX_BUF_LEN MAX(RESP_MSG_LEN, REPORT_MSG_LEN)

static uint8 tx_poll_msg[POLL_MSG_LEN] = {0x41, 0x88, 0,   0xCA, 0xDE, 'W',
                                          'A',  'V',  'E', 0x61, 0,    0};
static uint8 rx_resp_msg[RESP_MSG_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x50, 0, 0, 0, 0, 0, 0};
static uint8 tx_final_msg[FINAL_MSG_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x69,
    0,    0,    0, 0,    0,    0,   0,   0,   0,   0};
static uint8 rx_report_msg[REPORT_MSG_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE3, 0, 0, 0, 0, 0, 0};

static uint8 sequence_count = 0;
static uint8 rx_buffer[RX_BUF_LEN];

#if IS_ENABLED(CONFIG_UWB_LOGIC_CLK)
static uint32_t logic_clk = 0;
#define set_logic_clock_ts(buf)                                                \
    memcpy(buf + LOGIC_CLK_OFFSET, &logic_clk, LOGIC_CLOCK_OVERHEAD)
#define update_logic_clk() logic_clk++
#else
#define set_logic_clock_ts(buf) (void)0
#define update_logic_clk()      (void)0
#endif // IS_ENABLED(CONFIG_UWB_LOGIC_CLK)

int set_initializer_pan_id(uint16_t id) {
    CHECK_UWB_ACTIVE();

    memcpy(tx_poll_msg + PAN_ID_OFFSET, &id, PAN_ID_OVERHEAD);
    memcpy(rx_resp_msg + PAN_ID_OFFSET, &id, PAN_ID_OVERHEAD);
    memcpy(tx_final_msg + PAN_ID_OFFSET, &id, PAN_ID_OVERHEAD);
    memcpy(rx_report_msg + PAN_ID_OFFSET, &id, PAN_ID_OVERHEAD);

    return 0;
}

int set_initializer_id(uint16_t id) {
    CHECK_UWB_ACTIVE();

    memcpy(tx_poll_msg + SRC_OFFSET, &id, SRC_OVERHEAD);
    memcpy(rx_resp_msg + DEST_OFFSET, &id, DEST_OVERHEAD);
    memcpy(tx_final_msg + SRC_OFFSET, &id, SRC_OVERHEAD);
    memcpy(rx_report_msg + DEST_OFFSET, &id, DEST_OVERHEAD);

    return 0;
}

static void set_destination(uint16_t id) {
    memcpy(tx_poll_msg + DEST_OFFSET, &id, DEST_OVERHEAD);
    memcpy(rx_resp_msg + SRC_OFFSET, &id, SRC_OVERHEAD);
    memcpy(tx_final_msg + DEST_OFFSET, &id, DEST_OVERHEAD);
    memcpy(rx_report_msg + SRC_OFFSET, &id, SRC_OVERHEAD);
}

static int send_poll_message(void) {
    int status;

    tx_poll_msg[SEQ_CNT_OFFSET] = sequence_count;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
    status = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    if (status != DWT_SUCCESS) {
        LOG_WRN("Unable to send polling message");
        return -ETIMEDOUT;
    }

    UWB_WAIT(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    sequence_count++;

    return 0;
}

static int wait_response_and_respond_final(void) {
    int status, ret;
    uint32 frame_len;
    UNUSED uint8 rx_seq;
    uint64_t poll_tx_ts, resp_rx_ts;
    uint32 final_tx_time;
    uint64_t final_tx_ts;

    UWB_WAIT((status = dwt_read32bitreg(SYS_STATE_ID)) &
             (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR));

    if (!(status & SYS_STATUS_RXFCG)) {
        /* Clear RX error/timeout events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
//        LOG_WRN("Not able to receive response");
//        if (status & SYS_STATUS_RXPHE) {
//            LOG_WRN("SYS_STATUS_RXPHE");
//        }
//        if (status & SYS_STATUS_RXFCE) {
//            LOG_WRN("SYS_STATUS_RXFCE");
//        }
//        if (status & SYS_STATUS_RXRFSL) {
//            LOG_WRN("SYS_STATUS_RXRFSL");
//        }
//        if (status & SYS_STATUS_RXSFDTO) {
//            LOG_WRN("SYS_STATUS_RXSFDTO");
//        }
//        if (status & SYS_STATUS_AFFREJ) {
//            LOG_WRN("SYS_STATUS_AFFREJ");
//        }
//        if (status & SYS_STATUS_ALL_RX_TO) {
//            LOG_WRN("SYS_STATUS_LDEERR");
//        }
//        if (status & SYS_STATUS_ALL_RX_TO) {
//            LOG_WRN("SYS_STATUS_ALL_RX_TO (timeout)");
//        }
        return __LINE__;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    // TODO: Figure out if sequence counter is sufficient for the logic clock
    rx_seq = rx_buffer[SEQ_CNT_OFFSET];

    // Cleared to simplify validation of the frame
    rx_buffer[SEQ_CNT_OFFSET] = 0;

    if (memcmp(rx_buffer, rx_resp_msg, DW_BASE_LEN) != 0) {
        LOG_WRN("Response invalid");
        return -EINVAL;
    }
    LOG_WRN("Response received");

    poll_tx_ts = get_tx_timestamp_u64();
    resp_rx_ts = get_rx_timestamp_u64();

    final_tx_time =
            (resp_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(final_tx_time);

    final_tx_ts =
            (((uint64_t)(final_tx_time & UINT32_C(0xFFFFFFFE))) << 8) + TX_ANT_DLY;

    memcpy(tx_final_msg + FINAL_TX_POLL_TS_OFFSET, &poll_tx_ts,
           TIMESTAMP_OVERHEAD);
    memcpy(tx_final_msg + FINAL_RX_RESP_TS_OFFSET, &resp_rx_ts,
           TIMESTAMP_OVERHEAD);
    memcpy(tx_final_msg + FINAL_TX_FINAL_TS_OFFSET, &final_tx_ts,
           TIMESTAMP_OVERHEAD);

    tx_final_msg[SEQ_CNT_OFFSET] = sequence_count;
    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
    dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1);
    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if (ret != DWT_SUCCESS) {
        dwt_rxreset();
        LOG_WRN("Unable to transmit final message");
        return -ETIME;
    }
    LOG_WRN("Final message sent");

    UWB_WAIT(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    sequence_count++;

    return 0;
}

static int wait_for_report(void) {
    int status;

    UWB_WAIT((status = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR));

    if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        dwt_rxreset();
        return -ETIME;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
    return 0;
}

ALWAYS_INLINE static int read_report(double *distance) {
    uint32 frame_length, msg_tof_dtu;
    UNUSED uint8 seq_count;
    double tof;

    frame_length = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
    if (frame_length <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_length, 0);
    }

    seq_count = rx_buffer[SEQ_CNT_OFFSET];
    rx_buffer[SEQ_CNT_OFFSET] = 0;

    if (memcmp(rx_buffer, rx_report_msg, DW_BASE_LEN) != 0) {
        return -EINVAL;
    }

    memcpy(&msg_tof_dtu, rx_buffer + REPORT_TOF_OFFSET, TIMESTAMP_OVERHEAD);

    tof = msg_tof_dtu * DWT_TIME_UNITS;
    *distance = tof * SPEED_OF_LIGHT;
    return 0;
}

ALWAYS_INLINE static int ss_read_response(void) {
    uint32_t frame_length;
    uint8 rx_seq;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    frame_length = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
    if (frame_length <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_length, 0);
    }

    // TODO: Figure out if sequence counter is sufficient for the logic clock
    rx_seq = rx_buffer[SEQ_CNT_OFFSET];
    rx_buffer[SEQ_CNT_OFFSET] = 0;

    if (memcmp(rx_buffer, rx_resp_msg, DW_BASE_LEN) != 0) {
        return -EINVAL;
    }

    return 0;
}

ALWAYS_INLINE static int ss_calculate_distance(uint8_t channel,
                                               double *distance) {
    uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
    int64_t rtd_init, rtd_resp;
    double clock_offset_ratio, ppm_multiplier, tof;

    poll_tx_ts = dwt_readtxtimestamplo32();
    resp_rx_ts = dwt_readrxtimestamplo32();

    switch (channel) {
    case 1:
        ppm_multiplier = HERTZ_TO_PPM_MULTIPLIER_CHAN_1;
        break;
    case 2:
        ppm_multiplier = HERTZ_TO_PPM_MULTIPLIER_CHAN_2;
        break;
    case 3:
        ppm_multiplier = HERTZ_TO_PPM_MULTIPLIER_CHAN_3;
        break;
    case 5:
        ppm_multiplier = HERTZ_TO_PPM_MULTIPLIER_CHAN_5;
        break;
    default:
        return -EINVAL;
    }

    clock_offset_ratio = dwt_readcarrierintegrator() *
                         (FREQ_OFFSET_MULTIPLIER * ppm_multiplier / 1.0e6);

    memcpy(&poll_rx_ts, rx_buffer + SS_RESP_RX_POLL_TS_OFFSET,
           TIMESTAMP_OVERHEAD);
    memcpy(&resp_tx_ts, rx_buffer + SS_RESP_TX_RESP_TS_OFFSET,
           TIMESTAMP_OVERHEAD);

    rtd_init = resp_rx_ts - poll_tx_ts;
    rtd_resp = resp_tx_ts - poll_rx_ts;

    tof = (((double)rtd_init - (double)rtd_resp * (1 - clock_offset_ratio)) /
           2.0) *
          DWT_TIME_UNITS;
    *distance = tof * SPEED_OF_LIGHT;

    return 0;
}

int double_sided_init(uint16_t id, double *distance) {
    int ret;
    uint64_t poll_tx_ts, resp_rx_ts;

    if (distance == NULL) {
        return -EINVAL;
    }

    set_destination(id);

    if ((ret = send_poll_message()) < 0) {
        return ret;
    }

    if ((ret = wait_response_and_respond_final()) != 0) {
        LOG_WRN("Returned %" PRId32, ret);
        return ret;
    }

    if ((ret = wait_for_report()) < 0) {
        return ret;
    }

    if ((ret = read_report(distance)) < 0) {
        return ret;
    }

    return 0;
}

int single_sided_init(uint16_t id, double *distance, uint8_t channel) {
    int ret;

    if (distance == NULL) {
        return -EINVAL;
    }

    set_destination(id);

//    if ((ret = send_poll_message()) < 0) {
//        return ret;
//    }
//
//    if ((ret = wait_for_response()) < 0) {
//        return ret;
//    }
//
//    if ((ret = ss_read_response()) < 0) {
//        return ret;
//    }
//
//    if ((ret = ss_calculate_distance(channel, distance)) < 0) {
//        return ret;
//    }

    return 0;
}
