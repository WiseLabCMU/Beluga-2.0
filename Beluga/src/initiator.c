

#include "initiator.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "random.h"
#include <app_leds.h>
#include <string.h>
#include <utils.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(initializer_logger, LOG_LEVEL_INF);

K_SEM_DEFINE(k_sus_init, 0, 1);

// TODO: Build config for this
#define DW_LOGIC_CLOCK_OVERHEAD 0

#define DW_BASE_LENGTH          10
#define DW_TIME_OVERHEAD        4
#define DW_CRC_OVERHEAD         2

#define POLL_MSG_LEN                                                           \
    (DW_BASE_LENGTH + DW_LOGIC_CLOCK_OVERHEAD + DW_CRC_OVERHEAD)
#define RESP_MSG_LEN                                                           \
    (DW_BASE_LENGTH + DW_TIME_OVERHEAD + DW_LOGIC_CLOCK_OVERHEAD +             \
     DW_CRC_OVERHEAD)
#define FINAL_MSG_LEN                                                          \
    (DW_BASE_LENGTH + DW_TIME_OVERHEAD + DW_TIME_OVERHEAD +                    \
     DW_LOGIC_CLOCK_OVERHEAD + DW_CRC_OVERHEAD)
#define REPORT_MSG_LEN                                                         \
    (DW_BASE_LENGTH + DW_TIME_OVERHEAD + DW_LOGIC_CLOCK_OVERHEAD +             \
     DW_CRC_OVERHEAD)

#define RX_BUF_LEN                                                             \
    MAX(MAX(POLL_MSG_LEN, RESP_MSG_LEN), MAX(FINAL_MSG_LEN, REPORT_MSG_LEN))

static uint8 tx_poll_msg[POLL_MSG_LEN] = {0x41, 0x88, 0,   0xCA, 0xDE, 'W',
                                          'A',  'V',  'E', 0x21, 0,    0};
static uint8 rx_resp_msg[RESP_MSG_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x50, 0, 0, 0, 0, 0, 0};
static uint8 tx_final_msg[FINAL_MSG_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x69,
    0,    0,    0, 0,    0,    0,   0,   0,   0,   0};
static uint8 rx_report_msg[REPORT_MSG_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE3, 0, 0, 0, 0, 0, 0};

#define FC_OFFSET               0
#define FC_BYTES                sizeof(uint16_t)

#define SEQUENCE_COUNTER_OFFSET 2
#define SEQUENCE_COUNTER_BYTES  sizeof(uint8_t)

#define PAN_ID_OFFSET           3
#define PAN_ID_BYTES            sizeof(uint16_t)

#define DESTINATION_OFFSET      5
#define DESTINATION_BYTES       sizeof(uint16_t)

#define SOURCE_OFFSET           7
#define SOURCE_BYTES            sizeof(uint16_t)

#define FUNCTION_CODE_OFFSET    9
#define FUNCTION_CODE_BYTES     sizeof(uint8_t)

static uint8 sequence_count = 0;
static uint8 rx_buffer[RX_BUF_LEN];

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion
 * factor. 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the
 * receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 300
/* This is the delay from Frame RX timestamp to TX reply timestamp used for
 * calculating/setting the DW1000's delayed TX function. This includes the frame
 * length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT               8

#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN          4
#define RESP_MSG_POLL_RX_TS_IDX   10
#define RESP_MSG_RESP_TX_TS_IDX   14

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

#define CHECK_UWB_ACTIVE()                                                     \
    do {                                                                       \
        if (get_uwb_led_state() == LED_UWB_ON) {                               \
            return -EBUSY;                                                     \
        }                                                                      \
    } while (0)

#define UWB_WAIT(cond) while (!(cond))

#define GET_UWB_MSG_TIMESTAMP(ts, ts_buf, offset)                              \
    do {                                                                       \
        (ts) = 0;                                                              \
        for (int i = offset; i < 4; i++) {                                     \
            (ts) += (ts_buf)[i] << ((i - (offset)) * 8);                       \
        }                                                                      \
    } while (0)

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

static uint64_t get_tx_timestamp_u64(void) {
    uint64_t ts;
    GET_UWB_TIMESTAMP(tx, ts);
    return ts;
}

static uint64_t get_rx_timestamp_u64(void) {
    uint64_t ts;
    GET_UWB_TIMESTAMP(rx, ts);
    return ts;
}

static void final_msg_set_ts(uint8 *ts_field, uint64_t ts) {
    for (int i = 0; i < FINAL_MSG_TS_LEN; i++) {
        ts_field[i] = (uint8)ts;
        ts >>= 8;
    }
}

int set_initializer_pan_id(uint16_t id) {
    CHECK_UWB_ACTIVE();

    memcpy(tx_poll_msg + PAN_ID_OFFSET, &id, PAN_ID_BYTES);
    memcpy(rx_resp_msg + PAN_ID_OFFSET, &id, PAN_ID_BYTES);
    memcpy(tx_final_msg + PAN_ID_OFFSET, &id, PAN_ID_BYTES);
    memcpy(rx_report_msg + PAN_ID_OFFSET, &id, PAN_ID_BYTES);

    return 0;
}

int set_initializer_id(uint16_t id) {
    CHECK_UWB_ACTIVE();

    memcpy(tx_poll_msg + SOURCE_OFFSET, &id, SOURCE_BYTES);
    memcpy(rx_resp_msg + DESTINATION_OFFSET, &id, DESTINATION_BYTES);
    memcpy(tx_final_msg + SOURCE_OFFSET, &id, SOURCE_BYTES);
    memcpy(rx_report_msg + DESTINATION_OFFSET, &id, DESTINATION_BYTES);

    return 0;
}

static void set_destination(uint16_t id) {
    memcpy(tx_poll_msg + DESTINATION_OFFSET, &id, DESTINATION_BYTES);
    memcpy(rx_resp_msg + SOURCE_OFFSET, &id, SOURCE_BYTES);
    memcpy(tx_final_msg + DESTINATION_OFFSET, &id, DESTINATION_BYTES);
    memcpy(rx_report_msg + SOURCE_OFFSET, &id, SOURCE_BYTES);
}

ALWAYS_INLINE static int send_poll_message(void) {
    int status;

    tx_poll_msg[SEQUENCE_COUNTER_OFFSET] = sequence_count;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
    status = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    if (status != DWT_SUCCESS) {
        return -ETIMEDOUT;
    }

    UWB_WAIT(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    sequence_count++;

    return 0;
}

ALWAYS_INLINE static int wait_for_response(void) {
    int status;

    UWB_WAIT((status = dwt_read32bitreg(SYS_STATE_ID)) &
             (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR));

    if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
        /* Clear RX error/timeout events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();

        return -ETIME;
    }

    return 0;
}

ALWAYS_INLINE static int ds_read_response(uint64_t *poll_tx_ts,
                                          uint64_t *resp_rx_ts) {
    uint32 frame_len;
    UNUSED uint8 rx_seq;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    // TODO: Figure out if sequence counter is sufficient for the logic clock
    rx_seq = rx_buffer[SEQUENCE_COUNTER_OFFSET];

    // Cleared to simplify validation of the frame
    rx_buffer[SEQUENCE_COUNTER_OFFSET] = 0;

    if (memcmp(rx_buffer, rx_resp_msg, DW_BASE_LENGTH) != 0) {
        return -EINVAL;
    }

    *poll_tx_ts = get_tx_timestamp_u64();
    *resp_rx_ts = get_rx_timestamp_u64();

    return 0;
}

ALWAYS_INLINE static int send_final_message(uint64_t poll_tx_ts,
                                            uint64_t resp_rx_ts) {
    int ret;
    uint32 final_tx_time;
    uint64_t final_tx_ts;

    final_tx_time =
        (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(final_tx_time);

    final_tx_ts =
        (((uint64_t)(final_tx_time & UINT32_C(0xFFFFFFFE))) << 8) + TX_ANT_DLY;

    final_msg_set_ts(tx_final_msg + FINAL_MSG_POLL_TX_TS_IDX, poll_tx_ts);
    final_msg_set_ts(tx_final_msg + FINAL_MSG_RESP_RX_TS_IDX, resp_rx_ts);
    final_msg_set_ts(tx_final_msg + FINAL_MSG_FINAL_TX_TS_IDX, final_tx_ts);

    tx_final_msg[SEQUENCE_COUNTER_OFFSET] = sequence_count;
    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
    dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1);
    ret = dwt_starttx(DWT_START_TX_DELAYED);

    if (ret != DWT_SUCCESS) {
        dwt_rxreset();
        return -ETIME;
    }

    UWB_WAIT(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    sequence_count++;

    return 0;
}

ALWAYS_INLINE static int wait_for_report(void) {
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

    seq_count = rx_buffer[SEQUENCE_COUNTER_OFFSET];
    rx_buffer[SEQUENCE_COUNTER_OFFSET] = 0;

    if (memcmp(rx_buffer, rx_report_msg, DW_BASE_LENGTH) != 0) {
        return -EINVAL;
    }

    GET_UWB_MSG_TIMESTAMP(msg_tof_dtu, rx_buffer, RESP_MSG_POLL_RX_TS_IDX);
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
    rx_seq = rx_buffer[SEQUENCE_COUNTER_OFFSET];
    rx_buffer[SEQUENCE_COUNTER_OFFSET] = 0;

    if (memcmp(rx_buffer, rx_resp_msg, DW_BASE_LENGTH) != 0) {
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

    GET_UWB_MSG_TIMESTAMP(poll_rx_ts, rx_buffer, RESP_MSG_POLL_RX_TS_IDX);
    GET_UWB_MSG_TIMESTAMP(resp_tx_ts, rx_buffer, RESP_MSG_RESP_TX_TS_IDX);

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

    if ((ret = wait_for_response()) < 0) {
        return ret;
    }

    if ((ret = ds_read_response(&poll_tx_ts, &resp_rx_ts)) < 0) {
        return ret;
    }

    if ((ret = send_final_message(poll_tx_ts, resp_rx_ts)) < 0) {
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

    if ((ret = send_poll_message()) < 0) {
        return ret;
    }

    if ((ret = wait_for_response()) < 0) {
        return ret;
    }

    if ((ret = ss_read_response()) < 0) {
        return ret;
    }

    if ((ret = ss_calculate_distance(channel, distance)) < 0) {
        return ret;
    }

    return 0;
}
