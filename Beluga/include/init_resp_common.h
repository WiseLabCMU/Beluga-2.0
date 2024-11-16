//
// Created by tom on 11/15/24.
//

#ifndef BELUGA_INIT_RESP_COMMON_H
#define BELUGA_INIT_RESP_COMMON_H

#include <zephyr/kernel.h>

#define FC_OVERHEAD 2
#define SEQ_CNT_OVERHEAD 1
#define PAN_ID_OVERHEAD 2
#define DEST_OVERHEAD 2
#define SRC_OVERHEAD 2
#define FUNCTION_CODE_OVERHEAD 1
#define CRC_OVERHEAD 2

#if IS_ENABLED(CONFIG_UWB_LOGIC_CLK)
#define LOGIC_CLOCK_OVERHEAD 4
#else
#define LOGIC_CLOCK_OVERHEAD 0
#endif  // IS_ENABLED(CONFIG_UWB_LOGIC_CLK)

#define DW_HEADER_OVERHEAD (FC_OVERHEAD + SEQ_CNT_OVERHEAD + PAN_ID_OVERHEAD + DEST_OVERHEAD + SRC_OVERHEAD + FUNCTION_CODE_OVERHEAD + LOGIC_CLOCK_OVERHEAD)
#define DW_FRAME_OVERHEAD (DW_HEADER_OVERHEAD + CRC_OVERHEAD)

#define DW_BASE_LEN DW_HEADER_OVERHEAD
#define TIMESTAMP_OVERHEAD 4

#define CHECK_UWB_ACTIVE()                                                     \
    do {                                                                       \
        if (get_uwb_led_state() == LED_UWB_ON) {                               \
            return -EBUSY;                                                     \
        }                                                                      \
    } while (0)

#define UWB_WAIT(cond) while (!(cond))

// Header Offsets
#define FC_OFFSET 0
#define SEQ_CNT_OFFSET (FC_OFFSET + FC_OVERHEAD) // 2
#define PAN_ID_OFFSET (SEQ_CNT_OFFSET + SEQ_CNT_OVERHEAD) // 3
#define DEST_OFFSET (PAN_ID_OFFSET + PAN_ID_OVERHEAD) // 5
#define SRC_OFFSET (DEST_OFFSET + DEST_OVERHEAD) // 7
#define FUNCTION_CODE_OFFSET (SRC_OFFSET + SRC_OVERHEAD) //9
#define LOGIC_CLK_OFFSET (FUNCTION_CODE_OFFSET + FUNCTION_CODE_OVERHEAD) // 10
#define DW_BASE_PAYLOAD_OFFSET (LOGIC_CLK_OFFSET + LOGIC_CLOCK_OVERHEAD) // 10 or 14

#define UUS_TO_DWT_TIME 65536
#define POLL_TX_TO_RESP_RX_DLY_UUS 300
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
#define RESP_RX_TIMEOUT_UUS 2700
#define SPEED_OF_LIGHT 299702547

#endif //BELUGA_INIT_RESP_COMMON_H
