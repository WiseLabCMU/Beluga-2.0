/**
 * @file beluga_service_common.c
 *
 * @brief
 *
 * @date 5/1/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <services/beluga_service_common.h>
#include <zephyr/kernel.h>

#define BT_BELUGA_SVC_SYNC_TWR_BYTE        0
#define BT_BELUGA_SVC_SYNC_SFD_BYTE        BT_BELUGA_SVC_SYNC_TWR_BYTE
#define BT_BELUGA_SVC_SYNC_PULSE_RATE_BYTE BT_BELUGA_SVC_SYNC_TWR_BYTE
#define BT_BELUGA_SVC_SYNC_PHR_BYTE        BT_BELUGA_SVC_SYNC_TWR_BYTE
#define BT_BELUGA_SVC_SYNC_DATA_RATE_BYTE  BT_BELUGA_SVC_SYNC_TWR_BYTE
#define BT_BELUGA_SVC_SYNC_PAC_BYTE        BT_BELUGA_SVC_SYNC_TWR_BYTE

#define BT_BELUGA_SVC_SYNC_PREAMBLE_BYTE   1
#define BT_BELUGA_SVC_SYNC_CHANNEL_BYTE    BT_BELUGA_SVC_SYNC_PREAMBLE_BYTE

#define BT_BELUGA_SVC_SYNC_POWER_AMP_BYTE  2

#define BT_BELUGA_SVC_SYNC_TX_POWER_BYTE   3

#define TWR_SHIFT                          0
#define SFD_SHIFT                          1
#define PULSE_RATE_SHIFT                   2
#define PHR_SHIFT                          3
#define DATA_RATE_SHIFT                    4
#define PAC_SHIFT                          6

#define PREAMBLE_SHIFT                     0
#define CHANNEL_SHIFT                      3

#define POWER_AMP_SHIFT                    0

#define TWR_MASK                           UINT8_C(0x1)
#define SFD_MASK                           UINT8_C(0x1)
#define PULSE_RATE_MASK                    UINT8_C(0x1)
#define PHR_MASK                           UINT8_C(0x1)
#define DATA_RATE_MASK                     UINT8_C(0x3)
#define PAC_MASK                           UINT8_C(0x3)

#define PREAMBLE_MASK                      UINT8_C(0x7)
#define CHANNEL_MASK                       UINT8_C(0x7)

#define POWER_AMP_MASK                     UINT8_C(0x7)

#define SERIALIZE_UWB_PARAM_VAL(name_, buf_, val_)                             \
    do {                                                                       \
        (buf_)[BT_BELUGA_SVC_SYNC_##name_##_BYTE] &=                           \
            ~(name_##_MASK << name_##_SHIFT);                                  \
        (buf_)[BT_BELUGA_SVC_SYNC_##name_##_BYTE] |= (name_##_MASK & (val_))   \
                                                     << name_##_SHIFT;         \
    } while (0)
#define SERIALIZE_UWB_PARAM(name_, buf_, configs_)                             \
    SERIALIZE_UWB_PARAM_VAL(name_, buf_, (configs_)->name_)

int serialize_uwb_configurations(char *buf, size_t len,
                                 const struct beluga_uwb_params *configs) {
    uint16_t preamble;
    if (buf == NULL || len < BT_BELUGA_SVC_SYNC_PAYLOAD_SIZE) {
        return -EINVAL;
    }
    SERIALIZE_UWB_PARAM(TWR, buf, configs);
    SERIALIZE_UWB_PARAM(SFD, buf, configs);
    SERIALIZE_UWB_PARAM(PULSE_RATE, buf, configs);
    SERIALIZE_UWB_PARAM(PHR, buf, configs);
    SERIALIZE_UWB_PARAM(DATA_RATE, buf, configs);
    SERIALIZE_UWB_PARAM(PAC, buf, configs);

    switch (configs->PREAMBLE) {
    case 64:
        preamble = 0;
        break;
    case 128:
        preamble = 1;
        break;
    case 256:
        preamble = 2;
        break;
    case 512:
        preamble = 3;
        break;
    case 1024:
        preamble = 4;
        break;
    case 1536:
        preamble = 5;
        break;
    case 2048:
        preamble = 6;
        break;
    case 4096:
        preamble = 7;
        break;
    default:
        return -EBADMSG;
    }

    SERIALIZE_UWB_PARAM_VAL(PREAMBLE, buf, preamble);
    SERIALIZE_UWB_PARAM(CHANNEL, buf, configs);

    memcpy(buf + BT_BELUGA_SVC_SYNC_POWER_AMP_BYTE, &configs->POWER_AMP,
           sizeof(configs->POWER_AMP));
    memcpy(buf + BT_BELUGA_SVC_SYNC_TX_POWER_BYTE, &configs->TX_POWER,
           sizeof(configs->TX_POWER));

    return 0;
}
