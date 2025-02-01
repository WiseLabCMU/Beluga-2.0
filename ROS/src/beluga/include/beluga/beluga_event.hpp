/**
 * @file beluga_event.hpp
 *
 * @brief
 *
 * @date 1/30/25
 *
 * @author tom
 */

#ifndef BELUGA_SERIAL_BELUGA_EVENT_HPP
#define BELUGA_SERIAL_BELUGA_EVENT_HPP

#include <condition_variable>
#include <mutex>

namespace Beluga {
class Event {
  public:
    Event() = default;

    void wait();
    void set();
    void clear();
    bool is_set();

  private:
    std::mutex mtx;
    std::condition_variable cv;
    bool signal = false;
};
} // namespace Beluga

#endif // BELUGA_SERIAL_BELUGA_EVENT_HPP
