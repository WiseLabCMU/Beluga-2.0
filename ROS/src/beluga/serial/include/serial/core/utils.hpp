/**
 * @file utils.hpp
 *
 * @brief
 *
 * @date 1/28/25
 *
 * @author tom
 */

#ifndef BELUGA_FRAME_UTILS_HPP
#define BELUGA_FRAME_UTILS_HPP

#include <exception>

namespace SerialInternal {
class NotImplemented : public std::exception {
  public:
    NotImplemented() = default;

    const char *what() const noexcept override { return message; }

  private:
    const char *message = "Not implemented";
};
} // namespace SerialInternal

#endif // BELUGA_FRAME_UTILS_HPP
