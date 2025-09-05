/**
 * @file utils.hpp
 *
 * @brief Utility functions and classes.
 *
 * @date 1/28/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_SERIAL_UTILS_HPP
#define BELUGA_SERIAL_UTILS_HPP

#include <exception>

namespace SerialInternal {

/// Exception for indicating if something is not implemented
class NotImplemented : public std::exception {
  public:
    NotImplemented() = default;

    [[nodiscard]] const char *what() const noexcept override {
        return _message;
    }

  private:
    const char *_message = "Not implemented";
};
} // namespace SerialInternal

#endif // BELUGA_SERIAL_UTILS_HPP
