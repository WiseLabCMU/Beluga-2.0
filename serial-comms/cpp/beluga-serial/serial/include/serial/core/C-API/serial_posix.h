/**
 * @file serial_posix.h
 *
 * @brief The C-API implementation for serial on POSIX systems.
 *
 * @date 1/28/25
 *
 * @author tom
 */

#ifndef BELUGA_FRAME_SERIAL_POSIX_H
#define BELUGA_FRAME_SERIAL_POSIX_H

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>
#include <unistd.h>

#include <serial/core/C-API/serial_common.h>

/**
 * Configuration attributes for a serial port on a POSIX system.
 */
struct serial_posix_config {
    int fd;                     ///< File descriptor for the serial port
    enum BaudRate baudrate;     ///< Baud rate for the serial port
    enum Parity parity;         ///< Parity setting for the serial port
    enum ByteSize bytesize;     ///< Byte size for the serial port
    enum StopBits stopbits;     ///< Stop bits for the serial port
    bool xonxoff;               ///< Software flow control (XON/XOFF)
    bool rtscts;                ///< Hardware flow control (RTS/CTS)
    bool exclusive;             ///< Use exclusive access to the serial port
    int32_t inter_byte_timeout; ///< Timeout in seconds for noncanonical read
                                ///< (TIME)
};

/**
 * Opens a serial port in non-blocking mode with no terminal control.
 *
 * @param[in] port The path to the serial port device (e.g., "/dev/ttyUSB0")
 * @return File descriptor on success
 * @return -1 on error with errno set
 */
int open_port(const char *port);

/**
 * @brief Configures the serial port with the given attributes.
 * @param[in] config The configuration attributes for the serial port.
 * @return 0 on success
 * @return -EINVAL if config is NULL
 * @return negative error code on failure
 */
int configure_port(struct serial_posix_config *config);

/**
 * @Brief Reads data from a serial port with a timeout.
 * @param[in] fd The file descriptor of the serial port.
 * @param[in,out] buf The buffer to store the read data.
 * @param[in] nbytes The number of bytes to read.
 * @param[in] time_left The amount of time read should block waiting for the
 * file descriptor to become ready. If NULL, read will block indefinitely.
 * @return Number of bytes read on success
 * @return -1 with errno set on failure
 */
ssize_t read_port(int fd, uint8_t *buf, size_t nbytes,
                  struct timeval *time_left);

/**
 * @brief Writes data to a serial port.
 * @param[in] fd The file descriptor of the serial port.
 * @param[in] buf The buffer containing the data to write.
 * @param[in] nbytes The number of bytes to write.
 * @return The number of bytes written on success
 * @return -1 with errno set on failure
 */
ssize_t write_port(int fd, const uint8_t *buf, size_t nbytes);

/**
 * @brief Closes a serial port.
 * @param[in] fd The file descriptor of the serial port.
 * @return 0 on success
 * @return -1 with errno set on failure
 */
int close_port(int fd);

/**
 * @brief Gets the number of bytes available to read from a serial port.
 * @param[in] fd The file descriptor of the serial port.
 * @return The number of bytes available to read on success
 * @return -1 with errno set on failure
 */
ssize_t port_in_waiting(int fd);

/**
 * @brief Blocks until the file descriptor is ready for writing.
 * @param[in] fd The file descriptor of the serial port.
 * @param[in] timeout The amount of time to wait for the file descriptor to
 * become ready. If NULL, wait indefinitely.
 * @return 1 if all the bytes have been written
 * @return 0 if the timeout expired
 * @return -1 on error with errno set
 */
int select_write_port(int fd, struct timeval *timeout);

/**
 * @brief Flushes the output buffer of a serial port.
 * @param[in] fd The file descriptor of the serial port.
 * @return 0 on success
 * @return -1 with errno set on failure
 */
int port_flush(int fd);

/**
 * @brief Clears the input buffer of a serial port.
 * @param[in] fd The file descriptor of the serial port.
 * @return 0 on success
 * @return -1 with errno set on failure
 */
int port_reset_input(int fd);

/**
 * @brief Clears the output buffer of a serial port.
 * @param[in] fd The file descriptor of the serial port.
 * @return 0 on success
 * @return -1 with errno set on failure
 */
int port_reset_output(int fd);

/**
 * @brief Checks if the Clear To Send (CTS) signal is active.
 * @param[in] fd The file descriptor of the serial port.
 * @return 1 if CTS is active
 * @return 0 if CTS is inactive
 * @return -1 on error with errno set
 */
int port_cts(int fd);

/**
 * @brief Checks if the Data Set Ready (DSR) signal is active.
 * @param[in] fd The file descriptor of the serial port.
 * @return 1 if DSR is active
 * @return 0 if DSR is inactive
 * @return -1 on error with errno set
 */
int port_dsr(int fd);

/**
 * @brief Checks if the Ring Indicator (RI) signal is active.
 * @param[in] fd The file descriptor of the serial port.
 * @return 1 if RI is active
 * @return 0 if RI is inactive
 * @return -1 on error with errno set
 */
int port_ri(int fd);

/**
 * @brief Checks if the Carrier Detect (CD) signal is active.
 * @param[in] fd The file descriptor of the serial port.
 * @return 1 if CD is active
 * @return 0 if CD is inactive
 * @return -1 on error with errno set
 */
int port_cd(int fd);

/**
 * @brief Sets the Request To Send (RTS) signal to the specified state.
 * @param[in] fd The file descriptor of the serial port.
 * @param[in] state The state to set the RTS signal to (true for high, false for
 * low).
 * @return 0 on success
 * @return -1 on error with errno set
 */
int port_set_rts_state(int fd, bool state);

/**
 * @brief Sets the Data Terminal Ready (DTR) signal to the specified state.
 * @param[in] fd The file descriptor of the serial port.
 * @param[in] state The state to set the DTR signal to (true for high, false for
 * low).
 * @return 0 on success
 * @return -1 on error with errno set
 */
int port_set_dtr_state(int fd, bool state);

#if defined(__cplusplus)
}
#endif

#endif // BELUGA_FRAME_SERIAL_POSIX_H
