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
        if (!dsrdtr_) {
            update_dtr_state_();
        }
        if (!rtscts_) {
            update_rts_state_();
        }
    } catch (const SerialException &err) {
        int code = err.code();
        if (!(code == EINVAL || code == ENOTTY)) {
            throw;
        }
    }
}

void SerialPosix::open() {
    if (port_.empty()) {
        throw SerialException("Port must be configured before it can be used.");
    }
    if (is_open_) {
        throw SerialException("Port is already open");
    }

    _fd = open_port(port_.c_str());
    if (_fd < 0) {
        throw SerialException(errno, "could not open port " + port_);
    }

    try {
        _reconfigure_port_internal();
        _init_flow_control();
        _reset_input_buffer();
    } catch (...) {
        close();
        throw;
    }
    is_open_ = true;
}

void SerialPosix::_reconfigure_port_internal() {
    struct SerialPosixConfig config = {
        .fd = _fd,
        .baudrate = baudrate_,
        .parity = parity_,
        .bytesize = bytesize_,
        .stopbits = stopbits_,
        .xonxoff = xonxoff_,
        .rtscts = rtscts_,
        .exclusive = exclusive_,
        .inter_byte_timeout = inter_byte_timeout_,
    };

    int ret = configure_port(&config);
    if (ret < 0) {
        throw SerialException(-ret, "Configuration failed");
    }
}

void SerialPosix::reconfigure_port_() {
    if (!is_open_) {
        return;
    }
    _reconfigure_port_internal();
}

void SerialPosix::close() {
    if (is_open_) {
        if (_fd > -1) {
            close_port(_fd);
        }
        is_open_ = false;
    }
}

size_t SerialPosix::in_waiting() {
    ssize_t waiting = port_in_waiting(_fd);
    if (waiting < 0) {
        throw SerialException(-(int)waiting,
                              "Unable to get number of bytes in waiting");
    }
    return waiting;
}

size_t SerialPosix::read(std::vector<uint8_t> &b, size_t n) {
    size_t bytes_read = 0;

    if (!is_open_) {
        throw PortNotOpenError();
    }

    b.clear();

    Timeout timeout(timeout_);

    while (bytes_read < n) {
        ssize_t ret;
        size_t read_len = n - bytes_read;
        std::vector<uint8_t> buf(read_len);
        if (timeout.infinite()) {
            ret = read_port(_fd, buf.data(), read_len, NULL);
        } else {
            struct timeval tv = timeout.time_left_tv();
            ret = read_port(_fd, buf.data(), read_len, &tv);
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
    int ret = select_write_port(_fd, &tv);
    if (ret == 0) {
        throw SerialTimeoutException("Write timeout");
    } else if (ret < 0) {
        throw SerialException(errno, "select failed");
    }
}

void SerialPosix::_wait_write_blocking() const {
    int ret = select_write_port(_fd, NULL);
    if (ret == 0) {
        throw SerialException("write failed (select)");
    }
}

size_t SerialPosix::write(const std::vector<uint8_t> &b) {
    if (!is_open_) {
        throw PortNotOpenError();
    }
    std::vector<uint8_t> d = b;
    size_t tx_len = b.size();
    Timeout timeout(write_timeout_);
    while (tx_len > 0) {
        ssize_t n = write_port(_fd, d.data(), tx_len);

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
    if (!is_open_) {
        throw PortNotOpenError();
    }
    int ret = port_flush(_fd);
    if (ret < 0) {
        throw SerialException(errno, "Unable to flush");
    }
}

void SerialPosix::reset_input_buffer() {
    if (!is_open_) {
        throw PortNotOpenError();
    }
    _reset_input_buffer();
}

void SerialPosix::reset_output_buffer() {
    if (!is_open_) {
        throw PortNotOpenError();
    }
    _reset_output_buffer();
}

bool SerialPosix::cts() {
    if (!is_open_) {
        throw PortNotOpenError();
    }
    int ret = port_cts(_fd);
    if (ret < 0) {
        throw SerialException(errno, "Cannot get CTS");
    }
    return ret != 0;
}

bool SerialPosix::dsr() {
    if (!is_open_) {
        throw PortNotOpenError();
    }
    int ret = port_dsr(_fd);
    if (ret < 0) {
        throw SerialException(errno, "Cannot get DSR");
    }
    return ret != 0;
}

bool SerialPosix::ri() {
    if (!is_open_) {
        throw PortNotOpenError();
    }
    int ret = port_ri(_fd);
    if (ret < 0) {
        throw SerialException(errno, "Cannot get RI");
    }
    return ret != 0;
}

bool SerialPosix::cd() {
    if (!is_open_) {
        throw PortNotOpenError();
    }
    int ret = port_cd(_fd);
    if (ret < 0) {
        throw SerialException(errno, "Cannot get CD");
    }
    return ret != 0;
}

void SerialPosix::update_rts_state_() {
    int ret = port_set_rts_state(_fd, rts_state_);
    if (ret < 0) {
        throw SerialException(errno, "Unable to set RTS state");
    }
}

void SerialPosix::update_dtr_state_() {
    int ret = port_set_dtr_state(_fd, dtr_state_);
    if (ret < 0) {
        throw SerialException(errno, "Unable to set DTR state");
    }
}

void SerialPosix::_reset_input_buffer() const {
    int ret = port_reset_input(_fd);

    if (ret < 0) {
        throw SerialException(errno, "Input buffer flush failed");
    }
}

void SerialPosix::_reset_output_buffer() const {
    int ret = port_reset_output(_fd);

    if (ret < 0) {
        throw SerialException(errno, "Output buffer flush failed");
    }
}
} // namespace SerialInternal
