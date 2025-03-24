/**
 * @file serial.hpp
 *
 * @brief Serial port implementation for all systems.
 *
 * @note Only supports POSIX systems, like Linux and OSX at this time.
 *
 * @date 1/28/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
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

/// Serial port implementation for all systems.
class Serial : public SerialInternal::SerialPosix {
  public:
    /**
     * Initialize comm port object
     * @param[in] attr The serial attributes to initialize the port with
     */
    explicit Serial(const SerialAttributes &attr = SerialAttributes{})
        : SerialInternal::SerialPosix(attr) {}
    ~Serial() override = default;

    // Disallow copy and move
    Serial(const Serial &other) = delete;
    Serial(Serial &&other) = delete;
    Serial &operator=(const Serial &other) = delete;
    Serial &operator=(Serial &&other) = delete;
};
} // namespace Serial

#endif // BELUGA_FRAME_SERIAL_HPP
