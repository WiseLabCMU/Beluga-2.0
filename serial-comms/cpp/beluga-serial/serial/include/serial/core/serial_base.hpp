/**
 * @file serial_base.hpp
 *
 * @brief Base class and support functions used by various backends.
 *
 * @date 1/28/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
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

/// Base class for serial port exceptions
class SerialException : public std::exception {
  public:
    /**
     * Construct a SerialException with a message
     * @param[in] message Description of the error
     */
    explicit SerialException(const char *message) { _message = message; }

    /**
     * Construct a SerialException with a message
     * @param[in] message Description of the error
     */
    explicit SerialException(const std::string &message) { _message = message; }

    /**
     * Construct a SerialException with an error number and message
     * @param[in] errnum The error number. Usually from errno
     * @param[in] message The description of the error. This is tied to the
     * error number
     */
    SerialException(int errnum, const std::string &message) {
        _code = errnum;
        std::stringstream oss;
        oss << errnum << " " << message << ": " << strerror(errnum);
        _message = oss.str();
    }

    /**
     * Return the error code
     * @return Error code
     * @return 0 if not set
     */
    [[nodiscard]] int code() const noexcept { return _code; }

    /**
     * Return the error message
     * @return The error message as a C-style string
     */
    [[nodiscard]] const char *what() const noexcept override {
        return _message.c_str();
    }

  protected:
    int _code = 0;
    std::string _message;
};

/// Write timeouts give an exception
class SerialTimeoutException : public SerialException {
  public:
    /**
     * Construct a SerialTimeoutException with a message
     * @param[in] message The description of the timeout error
     */
    explicit SerialTimeoutException(const char *message)
        : SerialException(message) {}

    /**
     * Construct a SerialTimeoutException with a message
     * @param[in] message The description of the timeout error
     */
    explicit SerialTimeoutException(const std::string &message)
        : SerialException(message) {}
};

/// Port is not open
class PortNotOpenError : public SerialException {
  public:
    PortNotOpenError()
        : SerialException("Attempting to use a port that is not open") {}
};

/// Abstraction for timeout operations.
class Timeout {
  public:
    /**
     * Initializes a timeout with the given duration.
     * @param[in] timeout The duration of the timeout in milliseconds. If an
     * infinite timeout is desired (block forever), use milloseconds::max(). If
     * non-blocking behavior is desired, use milliseconds::zero().
     */
    explicit Timeout(const milliseconds &timeout);

    /**
     * Indicator if the timeout has expired
     * @return `true` if the timeout has expired
     * @return `false` otherwise
     */
    bool expired();

    /**
     * Return how much time is left in the timeout in milliseconds.
     * @return The time left in milliseconds
     */
    milliseconds time_left();

    /**
     * How much time is left in the timeout as a timeval.
     * @return The timeval representation of the time left
     */
    struct timeval time_left_tv();

    /**
     * Restart a timeout with a new duration
     * @param[in] duration The new duration of the timeout
     */
    void restart(const milliseconds &duration);

    /**
     * Attribute indicating if the timeout is infinite
     * @return `true` if the timeout is infinite
     * @return `false` otherwise
     */
    [[nodiscard]] bool infinite() const noexcept;

    /**
     * Attribute indicating if the timeout is non-blocking
     * @return `true` if the timeout is non-blocking
     * @return `false` otherwise
     */
    [[nodiscard]] bool non_blocking() const noexcept;

    /**
     * Get the duration of the timeout
     * @return The duration of the timeout in milliseconds
     */
    [[nodiscard]] milliseconds duration() const noexcept;

  private:
    milliseconds _duration{};
    std::chrono::steady_clock::time_point _target_time;
    bool _infinite;
    bool _non_blocking;
};

/**
 * @brief Serial port attributes
 */
struct SerialAttributes {
    std::string port;                           ///< The file name of the port
    enum BaudRate baudrate = BAUD_DEFAULT;      ///< The baudrate
    enum ByteSize bytesize = SIZE_DEFAULT;      ///< The number of data bits
    enum Parity parity = PARITY_DEFAULT;        ///< Parity
    enum StopBits stopbits = STOPBITS_DEFAULT;  ///< The number of stop bits
    milliseconds timeout = milliseconds::max(); ///< Read timeout
    bool xonxoff = false; ///< Software flow control (XON/XOFF)
    bool rtscts = false;  ///< Hardware flow control (RTS/CTS)
    milliseconds write_timeout = milliseconds::max(); ///< Write timeout
    bool dsrdtr = false;             ///< Hardware flow control (DSR/DTR)
    int32_t inter_byte_timeout = -1; ///< Inter-byte timeout in seconds
    bool exclusive = false;          ///< Open port exclusively
};

/// Serial port base class
class SerialBase {
  public:
    /**
     * Initializes comm port object.
     * @param[in] attr The serial attributes to initialize the port with
     */
    explicit SerialBase(const SerialAttributes &attr = SerialAttributes{});
    virtual ~SerialBase() = 0;

    /**
     * Get the current port setting
     * @return The current port setting
     */
    [[nodiscard]] std::string port() const noexcept;

    /**
     * Change the port. If closed, reconfiguration does not occur.
     * @param[in] portname The new port name.
     * @throw SerialException if reconfiguration fails
     */
    void port(const std::string &portname);

    /**
     * Get the current baudrate setting.
     * @return The current baudrate
     */
    [[nodiscard]] enum BaudRate baudrate() const noexcept;

    /**
     * Change the baudrate. If closed, reconfiguration does not occur.
     * @param[in] baud The new baudrate
     * @throws SerialException if reconfiguration fails
     */
    void baudrate(enum BaudRate baud);

    /**
     * Get the current byte size setting.
     * @return The current byte size.
     */
    [[nodiscard]] enum ByteSize bytesize() const noexcept;

    /**
     * Change the byte size. If closed, reconfiguration does not occur.
     * @param[in] size The new byte size
     * @throws SerialException if reconfiguration fails
     */
    void bytesize(enum ByteSize size);

    /**
     * Get the current exclusive access setting.
     * @return `true` if the port is opened exclusively
     */
    [[nodiscard]] bool exclusive() const noexcept;

    /**
     * Change the exclusive access setting. If closed, reconfiguration does not
     * occur.
     * @param[in] excl `true` to open the port exclusively
     * @throws SerialException if reconfiguration fails
     */
    void exclusive(bool excl);

    /**
     * Get the current parity setting.
     * @return The parity setting
     */
    [[nodiscard]] enum Parity parity() const noexcept;

    /**
     * Change the parity. If closed, reconfiguration does not occur.
     * @param[in] par The new parity setting
     * @throws SerialException if reconfiguration fails
     */
    void parity(enum Parity par);

    /**
     * Get the current stop bits setting.
     * @return The current stop bits setting
     */
    [[nodiscard]] enum StopBits stopbits() const noexcept;

    /**
     * Change the stop bits setting. If closed, reconfiguration does not occur.
     * @param[in] bits The new stop bits setting
     * @throws SerialException if reconfiguration fails
     */
    void stopbits(enum StopBits bits);

    /**
     * Get the current timeout setting.
     * @return The current timeout
     */
    [[nodiscard]] milliseconds timeout() const noexcept;

    /**
     * Change the timeout. If closed, reconfiguration does not occur.
     * @param[in] duration The new timeout duration
     * @throws SerialException if reconfiguration fails
     */
    void timeout(const milliseconds &duration);

    /**
     * Get the current write timeout setting.
     * @return The current write timeout
     */
    [[nodiscard]] milliseconds write_timeout() const noexcept;

    /**
     * Change the write timeout. If closed, reconfiguration does not occur.
     * @param[in] duration The new write timeout duration
     * @throws SerialException if reconfiguration fails
     */
    void write_timeout(const milliseconds &duration);

    /**
     * Get the current inter-byte timeout setting.
     * @return The current inter-byte timeout in seconds
     */
    [[nodiscard]] uint32_t inter_byte_timeout() const noexcept;

    /**
     * Change the inter-byte timeout. If closed, reconfiguration does not occur.
     * @param[in] ic_timeout The new inter-byte timeout in seconds
     * @throws SerialException if reconfiguration fails
     */
    void inter_byte_timeout(uint32_t ic_timeout);

    /**
     * Get the current XON/XOFF flow-control setting.
     * @return `true` if XON/XOFF flow control is enabled
     * @return `false` otherwise
     */
    [[nodiscard]] bool xonxoff() const noexcept;

    /**
     * Change the XON/XOFF flow-control setting. If closed, reconfiguration does
     * not occur.
     * @param[in] fc `true` to enable XON/XOFF flow control. `false` to disable.
     * @throws SerialException if reconfiguration fails
     */
    void xonxoff(bool fc);

    /**
     * Get the current RTS/CTS flow-control setting.
     * @return `true` if RTS/CTS flow control is enabled
     * @return `false` otherwise
     */
    [[nodiscard]] bool rtscts() const noexcept;

    /**
     * Change the RTS/CTS flow-control setting. If closed, reconfiguration does
     * not occur.
     * @param[in] fc `true` to enable RTS/CTS flow control. `false` to disable.
     * @throws SerialException if reconfiguration fails
     */
    void rtscts(bool fc);

    /**
     * Get the current DSR/DTR flow-control setting.
     * @return `true` if DSR/DTR flow control is enabled
     * @return `false` otherwise
     */
    [[nodiscard]] bool dsrdtr() const noexcept;

    /**
     * Change the DSR/DTR flow-control setting. If closed, reconfiguration does
     * not occur.
     * @param[in] fc `true` to enable DSR/DTR flow control. `false` to disable.
     * @throws SerialException if reconfiguration fails
     */
    void dsrdtr(bool fc);

    /**
     * Get the current RTS (Ready to Send) state.
     * @return `true` if RTS is enabled
     * @return `false` otherwise
     */
    [[nodiscard]] bool rts() const noexcept;

    /**
     * Change the RTS (Ready to Send) state.
     * @param[in] state `true` to enable RTS
     * @param[in] state `false` to disable RTS
     */
    void rts(bool state);

    /**
     * Get the current DTR (Data Terminal Ready) state.
     * @return `true` if DTR is enabled
     * @return `false` otherwise
     */
    [[nodiscard]] bool dtr() const noexcept;

    /**
     * Change the DTR (Data Terminal Ready) state.
     * @param[in] state `true` to enable DTR
     * @param[in] state `false` to disable DTR
     */
    void dtr(bool state);

    /// Indicates if the port is readable. Always returns true.
    static bool readable() noexcept;

    /// Indicates if the port is writable. Always returns true.
    static bool writable() noexcept;

    /// Indicates if the port is seekable. Always returns false.
    static bool seekable() noexcept;

    /**
     * Read all bytes currently available in the buffer of the OS.
     * @param[in,out] b The buffer to read bytes into
     * @return The number of bytes read
     * @throws PortNotOpenError if the port is not open
     * @throws SerialException if an error occurs
     */
    size_t read_all(std::vector<uint8_t> &b);

    /**
     * Read until the expected sequence is found (line feed by default), the
     * size is exceeded or until a timeout occurs.
     * @param[in,out] b The buffer to read bytes into
     * @param[in] expected The expected line ending sequence
     * @param[in] len The maximum number of bytes to read
     * @return The number of bytes read
     * @throws PortNotOpenError if the port is not open
     */
    size_t read_until(std::vector<uint8_t> &b,
                      const std::vector<uint8_t> &expected = {'\n'},
                      size_t len = 0);

    /**
     * Check if the port is closed
     * @return `true` if the port is closed
     * @return `false` otherwise
     */
    [[nodiscard]] bool is_closed() const noexcept;

    /**
     * Check if the port is open
     * @return `true` if the port is open
     * @return `false` otherwise
     */
    [[nodiscard]] bool is_open() const noexcept;

    /**
     * Open port with current settings.
     *
     * @note This is required to be overridden by the derived class.
     */
    virtual void open() = 0;

    /**
     * Close the port.
     *
     * @note This is required to be overridden by the derived class.
     */
    virtual void close() = 0;

    /**
     * Get the number of bytes in the input buffer.
     * @return The number of bytes available to read
     *
     * @note This is required to be overridden by the derived class.
     */
    virtual size_t in_waiting() = 0;

    /**
     * Read bytes from the port. If a timeout is set, it may return less
     * characters than requested. With no timeout, it will block until the
     * requested number of bytes are read.
     * @param[in,out] b The buffer to read bytes into
     * @param[in] n The maximum number of bytes to read
     * @return Number of bytes read
     *
     * @note This is required to be overridden by the derived class.
     */
    virtual size_t read(std::vector<uint8_t> &b, size_t n) = 0;

    /**
     * Output the given bytes over the serial port.
     * @param[in] b The bytes to transmit
     * @return The number of bytes written
     *
     * @note This is required to be overridden by the derived class.
     */
    virtual size_t write(const std::vector<uint8_t> &b) = 0;

    /**
     * Flush the output buffer.
     *
     * @note This is required to be overridden by the derived class.
     */
    virtual void flush() = 0;

    /**
     * Reset the input buffer, discarding all data that is in the buffer.
     *
     * @note This is required to be overridden by the derived class.
     */
    virtual void reset_input_buffer() = 0;

    /**
     * Reset the output buffer, discarding all data that is in the buffer.
     *
     * @note This is required to be overridden by the derived class.
     */
    virtual void reset_output_buffer() = 0;

    /**
     * Read the terminal status line: Clear To Send (CTS)
     * @return `true` if CTS is enabled
     *
     * @note This is required to be overridden by the derived class.
     */
    virtual bool cts() = 0;

    /**
     * Read the terminal status line: Data Set Ready (DSR)
     * @return `true` if DSR is enabled
     *
     * @note This is required to be overridden by the derived class.
     */
    virtual bool dsr() = 0;

    /**
     * Read the terminal status line: Ring Indicator (RI)
     * @return `true` if RI is enabled
     *
     * @note This is required to be overridden by the derived class.
     */
    virtual bool ri() = 0;

    /**
     * Read the terminal status line: Carrier Detect (CD)
     * @return `true` if CD is enabled
     *
     * @note This is required to be overridden by the derived class.
     */
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
