/**
 * @file serial_posix.hpp
 *
 * @brief
 *
 * @date 1/28/25
 *
 * @author tom
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
    explicit SerialPosix(const SerialAttributes &attr = SerialAttributes{})
        : SerialBase(attr) {
        if (!_port.empty()) {
            open();
        }
    }
    ~SerialPosix() override { close(); };

    void open() override;
    void close() override;
    size_t in_waiting() override;
    size_t read(std::vector<uint8_t> &b, size_t n) override;
    size_t write(const std::vector<uint8_t> &b) override;
    void flush() override;
    void reset_input_buffer() override;
    void reset_output_buffer() override;
    bool cts() override;
    bool dsr() override;
    bool ri() override;
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
