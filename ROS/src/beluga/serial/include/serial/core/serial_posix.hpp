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
class SerialPosix : public SerialBase {
  public:
    explicit SerialPosix(const std::string &port = "",
                         enum BaudRate baudrate = BAUD_115200,
                         enum ByteSize bytesize = SIZE_8,
                         enum Parity parity = PARITY_NONE,
                         enum StopBits stopbits = STOPBITS_1,
                         const std::chrono::milliseconds &timeout =
                             std::chrono::milliseconds::max(),
                         bool xonxoff = false, bool rtscts = false,
                         const std::chrono::milliseconds &write_timeout =
                             std::chrono::milliseconds::max(),
                         bool dsrdtr = false, int32_t inter_byte_timeout = -1,
                         bool exclusive = false)
        : SerialBase(port, baudrate, bytesize, parity, stopbits, timeout,
                     xonxoff, rtscts, write_timeout, dsrdtr, inter_byte_timeout,
                     exclusive) {
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
    size_t read_byte(uint8_t *byte) override;
    size_t write_byte(uint8_t byte) override;
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
