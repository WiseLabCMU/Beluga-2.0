/**
 * @file beluga_event.cpp
 *
 * @brief
 *
 * @date 1/30/25
 *
 * @author tom
 */

#include <beluga/beluga_event.hpp>

namespace BelugaSerial {
void Event::wait() {
    std::unique_lock<std::mutex> lock(_mtx);
    _cv.wait(lock, [this]() { return _signal; });
}

void Event::set() {
    std::lock_guard<std::mutex> lock(_mtx);
    _signal = true;
    _cv.notify_all();
}

void Event::clear() {
    std::lock_guard<std::mutex> lock(_mtx);
    _signal = false;
}

bool Event::is_set() {
    std::lock_guard<std::mutex> lock(_mtx);
    return _signal;
}
} // namespace BelugaSerial