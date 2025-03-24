/**
 * @file serial_common.h
 *
 * @brief Common definitions for serial communication.
 *
 * @date 1/28/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_FRAME_SERIAL_COMMON_H
#define BELUGA_FRAME_SERIAL_COMMON_H

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * Baud rate settings for serial communication.
 */
enum BaudRate {
    BAUD_0,                     ///< Hangup
    BAUD_50,                    ///< 50 baud
    BAUD_75,                    ///< 75 baud
    BAUD_110,                   ///< 110 baud
    BAUD_134,                   ///< 134 baud
    BAUD_150,                   ///< 150 baud
    BAUD_200,                   ///< 200 baud
    BAUD_300,                   ///< 300 baud
    BAUD_600,                   ///< 600 baud
    BAUD_1200,                  ///< 1200 baud
    BAUD_1800,                  ///< 1800 baud
    BAUD_2400,                  ///< 2400 baud
    BAUD_4800,                  ///< 4800 baud
    BAUD_9600,                  ///< 9600 baud
    BAUD_19200,                 ///< 19200 baud
    BAUD_38400,                 ///< 38400 baud
    BAUD_57600,                 ///< 57600 baud
    BAUD_115200,                ///< 115200 baud
    BAUD_230400,                ///< 230400 baud
    BAUD_460800,                ///< 460800 baud
    BAUD_500000,                ///< 500000 baud
    BAUD_576000,                ///< 576000 baud
    BAUD_921600,                ///< 921600 baud
    BAUD_1000000,               ///< 1000000 baud
    BAUD_1152000,               ///< 1152000 baud
    BAUD_1500000,               ///< 1500000 baud
    BAUD_2000000,               ///< 2000000 baud
    BAUD_2500000,               ///< 2500000 baud
    BAUD_3000000,               ///< 3000000 baud
    BAUD_3500000,               ///< 3500000 baud
    BAUD_4000000,               ///< 4000000 baud
    BAUD_DEFAULT = BAUD_115200, ///< Default baud rate (115200 baud)
    BAUD_INVALID,               ///< Last enumerator
};

/**
 * Parity settings for serial communication.
 */
enum Parity {
    PARITY_NONE,                  ///< No parity
    PARITY_EVEN,                  ///< Even parity
    PARITY_ODD,                   ///< Odd parity
    PARITY_MARK,                  ///< Mark parity
    PARITY_SPACE,                 ///< Space parity
    PARITY_DEFAULT = PARITY_NONE, ///< Default parity (no parity)
    PARITY_INVALID,               ///< Last enumerator
};

/**
 * Byte size settings for serial communication.
 */
enum ByteSize {
    SIZE_5,                ///< 5 data bits
    SIZE_6,                ///< 6 data bits
    SIZE_7,                ///< 7 data bits
    SIZE_8,                ///< 8 data bits
    SIZE_DEFAULT = SIZE_8, ///< Default byte size (8 data bits)
    SIZE_INVALID,          ///< Last enumerator
};

/**
 * Number of stop bits settings for serial communication.
 */
enum StopBits {
    STOPBITS_1,                    ///< 1 stop bit
    STOPBITS_1P5,                  ///< 1.5 stop bits
    STOPBITS_2,                    ///< 2 stop bits
    STOPBITS_DEFAULT = STOPBITS_1, ///< Default stop bits (1 stop bit)
    STOPBITS_INVALID,              ///< Last enumerator
};

#if defined(__cplusplus)
}
#endif

#endif // BELUGA_FRAME_SERIAL_COMMON_H
