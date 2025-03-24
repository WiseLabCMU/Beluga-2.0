/**
 * @file serial_posix.hpp
 *
 * @brief Backend for serial IO for POSIX compatible systems, like Linux and OSX
 *
 * @date 1/28/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_FRAME_SERIAL_POSIX_HPP
#define BELUGA_FRAME_SERIAL_POSIX_HPP

#include <chrono>
#include <serial/core/C-API/serial_common.h>
#include <serial/core/C-API/serial_posix.h>
#include <serial/core/serial_base.hpp>
#include <string>

namespace SerialInternal {

/**
 * Serial port class POSIX implementation. Serial port configuration is done
 * with termios and fcntl. Runs on Linux and many other Un*x like systems.
 */
class SerialPosix : public SerialBase {
  public:
    /**
     * Initializes comm port object for a POSIX system
     * @param[in] attr The serial attributes to initialize the port with
     */
    explicit SerialPosix(const SerialAttributes &attr = SerialAttributes{})
        : SerialBase(attr) {
        if (!_port.empty()) {
            open();
        }
    }

    /**
     * Destructor for the SerialPosix object. Closes the port if it is open.
     */
    ~SerialPosix() override { close(); };

    /**
     * Open port with current settings.
     * @throws SerialException if the port cannot be opened
     */
    void open() override;

    /**
     * Close the port.
     */
    void close() override;

    /**
     * Get the number of bytes in the input buffer.
     * @return The number of bytes available to read
     * @throws SerialException if an error occurs
     */
    size_t in_waiting() override;

    /**
     * Read bytes from the port. If a timeout is set, it may return less
     * characters than requested. With no timeout, it will block until the
     * requested number of bytes are read.
     * @param[in,out] b The buffer to read bytes into
     * @param[in] n The maximum number of bytes to read
     * @return Number of bytes read
     * @throws PortNotOpenError if the port is not open
     * @throws SerialException if an error occurs
     */
    size_t read(std::vector<uint8_t> &b, size_t n) override;

    /**
     * Output the given bytes over the serial port.
     * @param[in] b The bytes to transmit
     * @return The number of bytes written
     * @throws PortNotOpenError if the port is not open
     * @throws SerialTimeoutException if a write timeout occurs
     * @throws SerialException if an error occurs
     */
    size_t write(const std::vector<uint8_t> &b) override;

    /**
     * Flush the output buffer.
     * @throws PortNotOpenError if the port is not open
     * @throws SerialException if an error occurs
     */
    void flush() override;

    /**
     * Reset the input buffer, discarding all data that is in the buffer.
     * @throws PortNotOpenError if the port is not open
     * @throws SerialException if an error occurs
     */
    void reset_input_buffer() override;

    /**
     * Reset the output buffer, discarding all data that is in the buffer.
     * @throws PortNotOpenError if the port is not open
     * @throws SerialException if an error occurs
     */
    void reset_output_buffer() override;

    /**
     * Read the terminal status line: Clear To Send (CTS)
     * @return `true` if CTS is enabled
     */
    bool cts() override;

    /**
     * Read the terminal status line: Data Set Ready (DSR)
     * @return `true` if DSR is enabled
     */
    bool dsr() override;

    /**
     * Read the terminal status line: Ring Indicator (RI)
     * @return `true` if RI is enabled
     */
    bool ri() override;

    /**
     * Read the terminal status line: Carrier Detect (CD)
     * @return `true` if CD is enabled
     */
    bool cd() override;

  private:
    const size_t rw_chunk_size = 1024;
    int fd = -1;

    void _init_flow_control();
    void _reconfigure_port() override;
    void _update_rts_state() override;
    void _update_dtr_state() override;
    void _reconfigure_port_internal();
    void _wait_write_timed(Timeout &timeout) const;
    void _wait_write_blocking() const;

    void _reset_input_buffer() const;
    void _reset_output_buffer() const;
};
} // namespace SerialInternal

#endif // BELUGA_FRAME_SERIAL_POSIX_HPP
