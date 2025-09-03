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

SerialBase::SerialBase(const SerialAttributes &attr) {
    is_open_ = false;
    port_ = attr.port;
    baudrate_ = attr.baudrate;
    bytesize_ = attr.bytesize;
    parity_ = attr.parity;
    stopbits_ = attr.stopbits;
    timeout_ = attr.timeout;
    write_timeout_ = attr.write_timeout;
    xonxoff_ = attr.xonxoff;
    rtscts_ = attr.rtscts;
    dsrdtr_ = attr.dsrdtr;
    inter_byte_timeout_ = attr.inter_byte_timeout;
    exclusive_ = attr.exclusive;
    rts_state_ = true;
    dtr_state_ = true;
}

SerialBase::~SerialBase() = default;

std::string SerialBase::port() const noexcept { return port_; }

void SerialBase::port(const std::string &portname) {
    port_ = portname;
    reconfigure_port_();
}

enum BaudRate SerialBase::baudrate() const noexcept { return baudrate_; }

void SerialBase::baudrate(enum BaudRate baud) {
    baudrate_ = baud;
    reconfigure_port_();
}

enum ByteSize SerialBase::bytesize() const noexcept { return bytesize_; }

void SerialBase::bytesize(enum ByteSize size) {
    bytesize_ = size;
    reconfigure_port_();
}

bool SerialBase::exclusive() const noexcept { return exclusive_; }

void SerialBase::exclusive(bool excl) {
    exclusive_ = excl;
    reconfigure_port_();
}

enum Parity SerialBase::parity() const noexcept { return parity_; }

void SerialBase::parity(enum Parity par) {
    parity_ = par;
    reconfigure_port_();
}

enum StopBits SerialBase::stopbits() const noexcept { return stopbits_; }

void SerialBase::stopbits(enum StopBits bits) {
    stopbits_ = bits;
    reconfigure_port_();
}

milliseconds SerialBase::timeout() const noexcept { return timeout_; }

void SerialBase::timeout(const milliseconds &duration) { timeout_ = duration; }

milliseconds SerialBase::write_timeout() const noexcept {
    return write_timeout_;
}

void SerialBase::write_timeout(const milliseconds &duration) {
    write_timeout_ = duration;
}

uint32_t SerialBase::inter_byte_timeout() const noexcept {
    return inter_byte_timeout_;
}

void SerialBase::inter_byte_timeout(uint32_t ic_timeout) {
    inter_byte_timeout_ = (int32_t)ic_timeout;
    reconfigure_port_();
}

bool SerialBase::xonxoff() const noexcept { return xonxoff_; }

void SerialBase::xonxoff(bool fc) {
    xonxoff_ = fc;
    reconfigure_port_();
}

bool SerialBase::rtscts() const noexcept { return rtscts_; }

void SerialBase::rtscts(bool fc) {
    rtscts_ = fc;
    reconfigure_port_();
}

bool SerialBase::dsrdtr() const noexcept { return dsrdtr_; }

void SerialBase::dsrdtr(bool fc) { dsrdtr_ = fc; }

bool SerialBase::rts() const noexcept { return rts_state_; }

void SerialBase::rts(bool state) {
    rts_state_ = state;
    update_rts_state_();
}

bool SerialBase::dtr() const noexcept { return dtr_state_; }

void SerialBase::dtr(bool state) {
    dtr_state_ = state;
    update_dtr_state_();
}

bool SerialBase::readable() noexcept { return true; }

bool SerialBase::writable() noexcept { return true; }

bool SerialBase::seekable() noexcept { return false; }

size_t SerialBase::read_all(std::vector<uint8_t> &b) {
    return read(b, in_waiting());
}

size_t SerialBase::read_until(std::vector<uint8_t> &b,
                              const std::vector<uint8_t> &expected,
                              size_t len) {
    size_t byte_read;
    size_t bytes_read = 0;
    std::vector<uint8_t> temp;
    Timeout timeout(timeout_);

    while (true) {
        byte_read = read(temp, 1);
        if (byte_read > 0) {
            b.push_back(temp[0]);
            bytes_read++;
            if (bytes_read >= expected.size()) {
                if (std::equal(expected.begin(), expected.end(),
                               b.end() - (long)expected.size())) {
                    break;
                }
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

bool SerialBase::is_closed() const noexcept { return !is_open_; }

bool SerialBase::is_open() const noexcept { return is_open_; }
} // namespace SerialInternal
