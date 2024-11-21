//
// Created by tom on 11/21/24.
//

#ifndef BELUGA_INIT_RESP_COMMON_H
#define BELUGA_INIT_RESP_COMMON_H

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

#endif // BELUGA_INIT_RESP_COMMON_H
