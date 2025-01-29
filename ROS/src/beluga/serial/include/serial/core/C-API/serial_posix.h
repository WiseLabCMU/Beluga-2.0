/**
 * @file serial_posix.h
 *
 * @brief
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

struct serial_posix_config {
    int fd;
    enum BaudRate baudrate;
    enum Parity parity;
    enum ByteSize bytesize;
    enum StopBits stopbits;
    bool xonxoff;
    bool rtscts;
    bool exclusive;
    int32_t inter_byte_timeout;
};

int open_port(const char *port);
int configure_port(struct serial_posix_config *config);
ssize_t read_port(int fd, uint8_t *buf, size_t nbytes,
                  struct timeval *time_left);
ssize_t write_port(int fd, const uint8_t *buf, size_t nbytes);
int close_port(int fd);
ssize_t port_in_waiting(int fd);
int select_write_port(int fd, struct timeval *timeout);
int port_flush(int fd);
int port_reset_input(int fd);
int port_reset_output(int fd);
int port_cts(int fd);
int port_dsr(int fd);
int port_ri(int fd);
int port_cd(int fd);
int port_set_rts_state(int fd, bool state);
int port_set_dtr_state(int fd, bool state);

#if defined(__cplusplus)
}
#endif

#endif // BELUGA_FRAME_SERIAL_POSIX_H
