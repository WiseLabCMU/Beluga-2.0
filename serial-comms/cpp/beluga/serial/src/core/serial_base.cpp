/**
 * @file serial_base.cpp
 *
 * @brief
 *
 * @date 1/28/25
 *
 * @author tom
 */

#include <serial/core/serial_base.hpp>

namespace SerialInternal {

Timeout::Timeout(const std::chrono::milliseconds &timeout) {
    _infinite = timeout == std::chrono::milliseconds::max();
    _non_blocking = timeout == std::chrono::milliseconds::zero();
    _duration = timeout;
    if (!_infinite) {
        _target_time = std::chrono::steady_clock::now() + timeout;
    } else {
        _target_time = std::chrono::steady_clock::time_point::max();
    }
}

bool Timeout::expired() {
    return _target_time != std::chrono::steady_clock::time_point::max() &&
           time_left() <= std::chrono::milliseconds::zero();
}

std::chrono::milliseconds Timeout::time_left() {
    if (_non_blocking) {
        return std::chrono::milliseconds::zero();
    } else if (_infinite) {
        return std::chrono::milliseconds::max();
    }
    auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(
        _target_time - std::chrono::steady_clock::now());
    if (delta > _duration) {
        // Clock jumped
        restart(_duration);
        return _duration;
    }
    return std::max(std::chrono::milliseconds::zero(), delta);
}

struct timeval Timeout::time_left_tv() {
    std::chrono::milliseconds time_left_ = time_left();
    struct timeval tv = {
        .tv_sec = std::chrono::duration_cast<std::chrono::seconds>(time_left_)
                      .count(),
        .tv_usec =
            std::chrono::duration_cast<std::chrono::microseconds>(time_left_)
                .count() %
            1000000};
    return tv;
}

void Timeout::restart(const std::chrono::milliseconds &duration) {
    _duration = duration;
    _target_time = std::chrono::steady_clock::now() + _duration;
}

bool Timeout::infinite() const noexcept { return _infinite; }

bool Timeout::non_blocking() const noexcept { return _non_blocking; }

std::chrono::milliseconds Timeout::duration() const noexcept {
    return _duration;
}

SerialBase::SerialBase(const std::string &port, enum BaudRate baudrate,
                       enum ByteSize bytesize, enum Parity parity,
                       enum StopBits stopbits, const milliseconds &timeout,
                       bool xonxoff, bool rtscts,
                       const milliseconds &write_timeout, bool dsrdtr,
                       int32_t inter_byte_timeout, bool exclusive) {
    _is_open = false;
    _port = port;
    _baudrate = baudrate;
    _bytesize = bytesize;
    _parity = parity;
    _stopbits = stopbits;
    _timeout = timeout;
    _write_timeout = write_timeout;
    _xonxoff = xonxoff;
    _rtscts = rtscts;
    _dsrdtr = dsrdtr;
    _inter_byte_timeout = inter_byte_timeout;
    _exclusive = exclusive;
    _rts_state = true;
    _dtr_state = true;
}

SerialBase::~SerialBase() = default;

std::string SerialBase::port() const noexcept { return _port; }

void SerialBase::port(const std::string &portname) {
    _port = portname;
    _reconfigure_port();
}

enum BaudRate SerialBase::baudrate() const noexcept { return _baudrate; }

void SerialBase::baudrate(enum BaudRate baud) {
    _baudrate = baud;
    _reconfigure_port();
}

enum ByteSize SerialBase::bytesize() const noexcept { return _bytesize; }

void SerialBase::bytesize(enum ByteSize size) {
    _bytesize = size;
    _reconfigure_port();
}

bool SerialBase::exclusive() const noexcept { return _exclusive; }

void SerialBase::exclusive(bool excl) {
    _exclusive = excl;
    _reconfigure_port();
}

enum Parity SerialBase::parity() const noexcept { return _parity; }

void SerialBase::parity(enum Parity par) {
    _parity = par;
    _reconfigure_port();
}

enum StopBits SerialBase::stopbits() const noexcept { return _stopbits; }

void SerialBase::stopbits(enum StopBits bits) {
    _stopbits = bits;
    _reconfigure_port();
}

milliseconds SerialBase::timeout() const noexcept { return _timeout; }

void SerialBase::timeout(const milliseconds &duration) { _timeout = duration; }

milliseconds SerialBase::write_timeout() const noexcept {
    return _write_timeout;
}

void SerialBase::write_timeout(const milliseconds &duration) {
    _write_timeout = duration;
}

uint32_t SerialBase::inter_byte_timeout() const noexcept {
    return _inter_byte_timeout;
}

void SerialBase::inter_byte_timeout(uint32_t ic_timeout) {
    _inter_byte_timeout = ic_timeout;
    _reconfigure_port();
}

bool SerialBase::xonxoff() const noexcept { return _xonxoff; }

void SerialBase::xonxoff(bool fc) {
    _xonxoff = fc;
    _reconfigure_port();
}

bool SerialBase::rtscts() const noexcept { return _rtscts; }

void SerialBase::rtscts(bool fc) {
    _rtscts = fc;
    _reconfigure_port();
}

bool SerialBase::dsrdtr() const noexcept { return _dsrdtr; }

void SerialBase::dsrdtr(bool fc) { _dsrdtr = fc; }

bool SerialBase::rts() const noexcept { return _rts_state; }

void SerialBase::rts(bool state) {
    _rts_state = state;
    _update_rts_state();
}

bool SerialBase::dtr() const noexcept { return _dtr_state; }

void SerialBase::dtr(bool state) {
    _dtr_state = state;
    _update_dtr_state();
}

bool SerialBase::readable() noexcept { return true; }

bool SerialBase::writable() noexcept { return true; }

bool SerialBase::seekable() noexcept { return false; }

size_t SerialBase::read_all(std::vector<uint8_t> &b) {
    return read(b, in_waiting());
}

size_t SerialBase::read_until(std::vector<uint8_t> &b, uint8_t c, size_t len) {
    size_t byte_read;
    uint8_t byte;
    size_t bytes_read = 0;

    Timeout timeout(_timeout);

    while (true) {
        byte_read = read_byte(&byte);
        if (byte_read > 0) {
            b.push_back(byte);
            bytes_read++;
            if (byte == c) {
                break;
            }
            if (len != 0 && bytes_read >= len) {
                break;
            }
        } else {
            break;
        }

        if (timeout.expired()) {
            break;
        }
    }

    return bytes_read;
}

bool SerialBase::is_closed() const noexcept { return !_is_open; }

bool SerialBase::is_open() const noexcept { return _is_open; }
} // namespace SerialInternal
