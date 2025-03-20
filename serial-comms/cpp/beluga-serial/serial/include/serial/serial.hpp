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
using SerialInternal::SerialAttributes;
using SerialInternal::SerialException;
using SerialInternal::SerialTimeoutException;

class Serial : public SerialInternal::SerialPosix {
  public:
    explicit Serial(const SerialAttributes &attr = SerialAttributes{})
        : SerialInternal::SerialPosix(attr) {}
    ~Serial() override = default;

    Serial(const Serial &other) = delete;
    Serial(Serial &&other) = delete;
    Serial &operator=(const Serial &other) = delete;
    Serial &operator=(Serial &&other) = delete;
};
} // namespace Serial

#endif // BELUGA_FRAME_SERIAL_HPP
