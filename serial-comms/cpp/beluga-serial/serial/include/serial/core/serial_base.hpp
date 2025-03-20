/**
 * @file serial_base.hpp
 *
 * @brief
 *
 * @date 1/28/25
 *
 * @author tom
 */

#ifndef BELUGA_FRAME_SERIAL_BASE_HPP
#define BELUGA_FRAME_SERIAL_BASE_HPP

#include <chrono>
#include <cstring>
#include <exception>
#include <serial/core/C-API/serial_common.h>
#include <sstream>
#include <string>
#include <vector>

namespace SerialInternal {
using std::chrono::milliseconds;

class SerialException : public std::exception {
  public:
    explicit SerialException(const char *message) { _message = message; }
    explicit SerialException(const std::string &message) { _message = message; }
    SerialException(int errnum, const std::string &message) {
        _code = errnum;
        std::stringstream oss;
        oss << errnum << " " << message << ": " << strerror(errnum);
        _message = oss.str();
    }

    [[nodiscard]] int code() const noexcept { return _code; }
    [[nodiscard]] const char *what() const noexcept override {
        return _message.c_str();
    }

  protected:
    int _code = 0;
    std::string _message;
};

class SerialTimeoutException : public SerialException {
  public:
    explicit SerialTimeoutException(const char *message)
        : SerialException(message) {}
    explicit SerialTimeoutException(const std::string &message)
        : SerialException(message) {}
};

class PortNotOpenError : public SerialException {
  public:
    PortNotOpenError()
        : SerialException("Attempting to use a port that is not open") {}
};

class Timeout {
  public:
    explicit Timeout(const milliseconds &timeout);

    bool expired();
    milliseconds time_left();
    struct timeval time_left_tv();
    void restart(const milliseconds &duration);

    [[nodiscard]] bool infinite() const noexcept;
    [[nodiscard]] bool non_blocking() const noexcept;
    [[nodiscard]] milliseconds duration() const noexcept;

  private:
    milliseconds _duration{};
    std::chrono::steady_clock::time_point _target_time;
    bool _infinite;
    bool _non_blocking;
};

struct SerialAttributes {
    std::string port;
    enum BaudRate baudrate = BAUD_DEFAULT;
    enum ByteSize bytesize = SIZE_DEFAULT;
    enum Parity parity = PARITY_DEFAULT;
    enum StopBits stopbits = STOPBITS_DEFAULT;
    milliseconds timeout = milliseconds::max();
    bool xonxoff = false;
    bool rtscts = false;
    milliseconds write_timeout = milliseconds::max();
    bool dsrdtr = false;
    int32_t inter_byte_timeout = -1;
    bool exclusive = false;
};

class SerialBase {
  public:
    explicit SerialBase(const SerialAttributes &attr = SerialAttributes{});
    virtual ~SerialBase() = 0;

    // Setters/Getters
    [[nodiscard]] std::string port() const noexcept;
    void port(const std::string &portname);

    [[nodiscard]] enum BaudRate baudrate() const noexcept;
    void baudrate(enum BaudRate baud);

    [[nodiscard]] enum ByteSize bytesize() const noexcept;
    void bytesize(enum ByteSize);

    [[nodiscard]] bool exclusive() const noexcept;
    void exclusive(bool excl);

    [[nodiscard]] enum Parity parity() const noexcept;
    void parity(enum Parity par);

    [[nodiscard]] enum StopBits stopbits() const noexcept;
    void stopbits(enum StopBits bits);

    [[nodiscard]] milliseconds timeout() const noexcept;
    void timeout(const milliseconds &duration);

    [[nodiscard]] milliseconds write_timeout() const noexcept;
    void write_timeout(const milliseconds &duration);

    [[nodiscard]] uint32_t inter_byte_timeout() const noexcept;
    void inter_byte_timeout(uint32_t ic_timeout);

    [[nodiscard]] bool xonxoff() const noexcept;
    void xonxoff(bool fc);

    [[nodiscard]] bool rtscts() const noexcept;
    void rtscts(bool fc);

    [[nodiscard]] bool dsrdtr() const noexcept;
    void dsrdtr(bool fc);

    [[nodiscard]] bool rts() const noexcept;
    void rts(bool state);

    [[nodiscard]] bool dtr() const noexcept;
    void dtr(bool state);

    // Constant Attributes
    static bool readable() noexcept;
    static bool writable() noexcept;
    static bool seekable() noexcept;

    // Common utility functions
    size_t read_all(std::vector<uint8_t> &b);
    size_t read_until(std::vector<uint8_t> &b, uint8_t c = '\n',
                      size_t len = 0);
    [[nodiscard]] bool is_closed() const noexcept;
    [[nodiscard]] bool is_open() const noexcept;

    // Platform defined functions
    virtual void open() = 0;
    virtual void close() = 0;
    virtual size_t in_waiting() = 0;
    virtual size_t read(std::vector<uint8_t> &b, size_t n) = 0;
    virtual size_t write(const std::vector<uint8_t> &b) = 0;
    virtual size_t read_byte(uint8_t *byte) = 0;
    virtual size_t write_byte(uint8_t byte) = 0;
    virtual void flush() = 0;
    virtual void reset_input_buffer() = 0;
    virtual void reset_output_buffer() = 0;
    virtual bool cts() = 0;
    virtual bool dsr() = 0;
    virtual bool ri() = 0;
    virtual bool cd() = 0;

  protected:
    // Protected Attributes
    bool _is_open;
    std::string _port;
    enum BaudRate _baudrate;
    enum ByteSize _bytesize;
    enum Parity _parity;
    enum StopBits _stopbits;
    milliseconds _timeout{};
    milliseconds _write_timeout{};
    bool _xonxoff;
    bool _rtscts;
    bool _dsrdtr;
    int32_t _inter_byte_timeout;
    bool _rts_state;
    bool _dtr_state;
    bool _exclusive;

    // Internal Routines

    // Platform defined internal routines
    virtual void _reconfigure_port() = 0;
    virtual void _update_rts_state() = 0;
    virtual void _update_dtr_state() = 0;
};
} // namespace SerialInternal

#endif // BELUGA_FRAME_SERIAL_BASE_HPP
