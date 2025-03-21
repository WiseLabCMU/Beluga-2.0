/**
 * @file beluga_event.hpp
 *
 * @brief Event class for synchronization
 *
 * @date 1/30/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_SERIAL_BELUGA_EVENT_HPP
#define BELUGA_SERIAL_BELUGA_EVENT_HPP

#include <condition_variable>
#include <mutex>

namespace BelugaSerial {

/// Event class for synchronization
class Event {
  public:
    Event() = default;

    /**
     * Wait until the event is set
     */
    void wait();

    /**
     * Set the event
     */
    void set();

    /**
     * Clear the event
     */
    void clear();

    /**
     * Check if the event is set
     * @return `true` if the event is set, `false` otherwise
     */
    bool is_set();

  private:
    std::mutex mtx;
    std::condition_variable cv;
    bool signal = false;
};
} // namespace BelugaSerial

#endif // BELUGA_SERIAL_BELUGA_EVENT_HPP
