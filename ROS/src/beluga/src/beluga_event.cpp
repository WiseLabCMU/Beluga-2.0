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
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this]() { return signal; });
}

void Event::set() {
    std::lock_guard<std::mutex> lock(mtx);
    signal = true;
    cv.notify_all();
}

void Event::clear() {
    std::lock_guard<std::mutex> lock(mtx);
    signal = false;
}

bool Event::is_set() {
    std::lock_guard<std::mutex> lock(mtx);
    return signal;
}
} // namespace BelugaSerial