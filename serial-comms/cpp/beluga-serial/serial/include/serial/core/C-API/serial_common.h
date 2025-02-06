/**
 * @file serial_common.h
 *
 * @brief
 *
 * @date 1/28/25
 *
 * @author tom
 */

#ifndef BELUGA_FRAME_SERIAL_COMMON_H
#define BELUGA_FRAME_SERIAL_COMMON_H

#if defined(__cplusplus)
extern "C" {
#endif

enum BaudRate {
    BAUD_0,
    BAUD_50,
    BAUD_75,
    BAUD_110,
    BAUD_134,
    BAUD_150,
    BAUD_200,
    BAUD_300,
    BAUD_600,
    BAUD_1200,
    BAUD_1800,
    BAUD_2400,
    BAUD_4800,
    BAUD_9600,
    BAUD_19200,
    BAUD_38400,
    BAUD_57600,
    BAUD_115200,
    BAUD_230400,
    BAUD_460800,
    BAUD_500000,
    BAUD_576000,
    BAUD_921600,
    BAUD_1000000,
    BAUD_1152000,
    BAUD_1500000,
    BAUD_2000000,
    BAUD_2500000,
    BAUD_3000000,
    BAUD_3500000,
    BAUD_4000000,
    BAUD_DEFAULT = BAUD_115200,
    BAUD_INVALID,
};

enum Parity {
    PARITY_NONE,
    PARITY_EVEN,
    PARITY_ODD,
    PARITY_MARK,
    PARITY_SPACE,
    PARITY_DEFAULT = PARITY_NONE,
    PARITY_INVALID,
};

enum ByteSize {
    SIZE_5,
    SIZE_6,
    SIZE_7,
    SIZE_8,
    SIZE_DEFAULT = SIZE_8,
    SIZE_INVALID,
};

enum StopBits {
    STOPBITS_1,
    STOPBITS_1P5,
    STOPBITS_2,
    STOPBITS_DEFAULT = STOPBITS_1,
    STOPBITS_INVALID,
};

#if defined(__cplusplus)
}
#endif

#endif // BELUGA_FRAME_SERIAL_COMMON_H
