/**
 * @file serial_posix.cpp
 *
 * @brief
 *
 * @date 1/28/25
 *
 * @author tom
 */

#include <cerrno>
#include <serial/core/C-API/serial_posix.h>
#include <serial/core/serial_posix.hpp>
#include <serial/core/utils.hpp>

namespace SerialInternal {
void SerialPosix::_init_flow_control() {
    try {
        if (!_dsrdtr) {
            _update_dtr_state();
        }
        if (!_rtscts) {
            _update_rts_state();
        }
    } catch (const SerialException &err) {
        int code = err.code();
        if (!(code == EINVAL || code == ENOTTY)) {
            throw;
        }
    }
}

void SerialPosix::open() {
    if (_port.empty()) {
        throw SerialException("Port must be configured before it can be used.");
    }
    if (_is_open) {
        throw SerialException("Port is already open");
    }

    fd = open_port(_port.c_str());
    if (fd < 0) {
        throw SerialException(errno, "could not open port " + _port);
    }

    try {
        _reconfigure_port_internal();
        _init_flow_control();
        _reset_input_buffer();
    } catch (...) {
        close();
        throw;
    }
    _is_open = true;
}

void SerialPosix::_reconfigure_port_internal() {
    struct serial_posix_config config = {
        .fd = fd,
        .baudrate = _baudrate,
        .parity = _parity,
        .bytesize = _bytesize,
        .stopbits = _stopbits,
        .xonxoff = _xonxoff,
        .rtscts = _rtscts,
        .exclusive = _exclusive,
        .inter_byte_timeout = _inter_byte_timeout,
    };

    int ret = configure_port(&config);
    if (ret < 0) {
        throw SerialException(-ret, "Configuration failed");
    }
}

void SerialPosix::_reconfigure_port() {
    if (!_is_open) {
        return;
    }
    _reconfigure_port_internal();
}

void SerialPosix::close() {
    if (_is_open) {
        if (fd > -1) {
            close_port(fd);
        }
        _is_open = false;
    }
}

size_t SerialPosix::in_waiting() {
    ssize_t waiting = port_in_waiting(fd);
    if (waiting < 0) {
        throw SerialException(-(int)waiting,
                              "Unable to get number of bytes in waiting");
    }
    return waiting;
}

size_t SerialPosix::read(std::vector<uint8_t> &b, size_t n) {
    size_t bytes_read = 0;

    if (!_is_open) {
        throw PortNotOpenError();
    }

    b.clear();

    Timeout timeout(_timeout);

    while (bytes_read < n) {
        ssize_t ret;
        size_t read_len = n - bytes_read;
        std::vector<uint8_t> buf(read_len);
        if (timeout.infinite()) {
            ret = read_port(fd, buf.data(), read_len, NULL);
        } else {
            struct timeval tv = timeout.time_left_tv();
            ret = read_port(fd, buf.data(), read_len, &tv);
        }
        if (ret < 0) {
            int err = errno;
            if (!(err == EAGAIN || err == EALREADY || err == EWOULDBLOCK ||
                  err == EINPROGRESS || err == EINTR || err == ETIMEDOUT)) {
                throw SerialException(errno, "read failed");
            }
        } else {
            if (ret == 0) {
                throw SerialException(
                    -ENODEV,
                    "device reports readiness to read but returned no data "
                    "(device disconnected or multiple access on port?)");
            }
            b.insert(b.end(), buf.begin(), buf.begin() + ret);
            bytes_read += ret;
        }

        if (timeout.expired()) {
            break;
        }
    }

    return bytes_read;
}

void SerialPosix::_wait_write_timed(Timeout &timeout) const {
    if (timeout.expired()) {
        throw SerialTimeoutException("Write timeout");
    }
    struct timeval tv = timeout.time_left_tv();
    int ret = select_write_port(fd, &tv);
    if (ret == 0) {
        throw SerialTimeoutException("Write timeout");
    } else if (ret < 0) {
        throw SerialException(errno, "select failed");
    }
}

void SerialPosix::_wait_write_blocking() const {
    int ret = select_write_port(fd, NULL);
    if (ret == 0) {
        throw SerialException("write failed (select)");
    }
}

size_t SerialPosix::write(const std::vector<uint8_t> &b) {
    if (!_is_open) {
        throw PortNotOpenError();
    }
    std::vector<uint8_t> d = b;
    size_t tx_len = b.size();
    Timeout timeout(_write_timeout);
    while (tx_len > 0) {
        ssize_t n = write_port(fd, d.data(), tx_len);

        if (n < 0) {
            int err = errno;
            if (!(err == EAGAIN || err == EALREADY || err == EWOULDBLOCK ||
                  err == EINPROGRESS || err == EINTR)) {
                throw SerialException(err, "write failed");
            }
            n = 0;
        }

        if (timeout.non_blocking()) {
            return n;
        } else if (!timeout.infinite()) {
            _wait_write_timed(timeout);
        } else {
            _wait_write_blocking();
        }

        d.erase(d.begin(), d.begin() + n);
        tx_len -= n;
    }

    return b.size() - d.size();
}

void SerialPosix::flush() {
    if (!_is_open) {
        throw PortNotOpenError();
    }
    int ret = port_flush(fd);
    if (ret < 0) {
        throw SerialException(errno, "Unable to flush");
    }
}

void SerialPosix::reset_input_buffer() {
    if (!_is_open) {
        throw PortNotOpenError();
    }
    _reset_input_buffer();
}

void SerialPosix::reset_output_buffer() {
    if (!_is_open) {
        throw PortNotOpenError();
    }
    _reset_output_buffer();
}

bool SerialPosix::cts() {
    if (!_is_open) {
        throw PortNotOpenError();
    }
    int ret = port_cts(fd);
    if (ret < 0) {
        throw SerialException(errno, "Cannot get CTS");
    }
    return ret != 0;
}

bool SerialPosix::dsr() {
    if (!_is_open) {
        throw PortNotOpenError();
    }
    int ret = port_dsr(fd);
    if (ret < 0) {
        throw SerialException(errno, "Cannot get DSR");
    }
    return ret != 0;
}

bool SerialPosix::ri() {
    if (!_is_open) {
        throw PortNotOpenError();
    }
    int ret = port_ri(fd);
    if (ret < 0) {
        throw SerialException(errno, "Cannot get RI");
    }
    return ret != 0;
}

bool SerialPosix::cd() {
    if (!_is_open) {
        throw PortNotOpenError();
    }
    int ret = port_cd(fd);
    if (ret < 0) {
        throw SerialException(errno, "Cannot get CD");
    }
    return ret != 0;
}

void SerialPosix::_update_rts_state() {
    int ret = port_set_rts_state(fd, _rts_state);
    if (ret < 0) {
        throw SerialException(errno, "Unable to set RTS state");
    }
}

void SerialPosix::_update_dtr_state() {
    int ret = port_set_dtr_state(fd, _dtr_state);
    if (ret < 0) {
        throw SerialException(errno, "Unable to set DTR state");
    }
}

void SerialPosix::_reset_input_buffer() const {
    int ret = port_reset_input(fd);

    if (ret < 0) {
        throw SerialException(errno, "Input buffer flush failed");
    }
}

void SerialPosix::_reset_output_buffer() const {
    int ret = port_reset_output(fd);

    if (ret < 0) {
        throw SerialException(errno, "Output buffer flush failed");
    }
}
} // namespace SerialInternal
