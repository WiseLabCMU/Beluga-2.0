/**
 * @file serial.hpp
 *
 * @brief
 *
 * @date 1/28/25
 *
 * @author tom
 */

#ifndef BELUGA_FRAME_SERIAL_HPP
#define BELUGA_FRAME_SERIAL_HPP

#include <serial/core/serial_posix.hpp>

namespace Serial {

using SerialInternal::milliseconds;
using SerialInternal::PortNotOpenError;
using SerialInternal::SerialException;
using SerialInternal::SerialTimeoutException;

class Serial : public SerialInternal::SerialPosix {
  public:
    explicit Serial(const std::string &port = "",
                    enum BaudRate baudrate = BAUD_115200,
                    enum ByteSize bytesize = SIZE_8,
                    enum Parity parity = PARITY_NONE,
                    enum StopBits stopbits = STOPBITS_1,
                    const milliseconds &timeout = milliseconds::max(),
                    bool xonxoff = false, bool rtscts = false,
                    const milliseconds &write_timeout = milliseconds::max(),
                    bool dsrdtr = false, int32_t inter_byte_timeout = -1,
                    bool exclusive = false)
        : SerialInternal::SerialPosix(
              port, baudrate, bytesize, parity, stopbits, timeout, xonxoff,
              rtscts, write_timeout, dsrdtr, inter_byte_timeout, exclusive) {}
    ~Serial() override = default;

    Serial(const Serial &other) = delete;
    Serial(Serial &&other) = delete;
    Serial &operator=(const Serial &other) = delete;
    Serial &operator=(Serial &&other) = delete;
};
} // namespace Serial

#endif // BELUGA_FRAME_SERIAL_HPP
