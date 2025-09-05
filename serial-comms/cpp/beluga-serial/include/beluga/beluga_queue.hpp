/**
 * @file beluga_queue.hpp
 *
 * @brief Thread safe implementation of a queue
 *
 * @date 1/30/25
 *
 * @authro Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */
#ifndef BELUGA_SERIAL_BELUGA_QUEUE_HPP
#define BELUGA_SERIAL_BELUGA_QUEUE_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <mutex>
#include <string>

namespace BelugaSerial {

/// Exception class for BelugaQueue
class BelugaQueueException : public std::exception {
  public:
    /**
     * @brief Enum for queue exception reasons
     */
    enum QueueExceptionReason { QUEUE_EMPTY, QUEUE_FULL, QUEUE_TIMEOUT };

    /**
     * @brief Constructor for BelugaQueueException
     *
     * @param[in] reason The reason for the exception
     */
    explicit BelugaQueueException(const QueueExceptionReason &reason)
        : _reason(reason) {
        switch (reason) {
        case QUEUE_EMPTY:
            _what = "Queue Empty";
            break;
        case QUEUE_FULL:
            _what = "Queue Full";
            break;
        case QUEUE_TIMEOUT:
            _what = "Queue Timed Out";
            break;
        default:
            _what = "Unknown Queue Error";
            break;
        }
    }

    /**
     * The reason for the exception as a string
     * @return The error reason
     */
    [[nodiscard]] const char *what() const noexcept override {
        return _what.c_str();
    }

    /**
     * @brief Get the reason for the exception
     * @return The reason for the exception
     */
    [[nodiscard]] QueueExceptionReason reason() const noexcept {
        return _reason;
    }

  private:
    QueueExceptionReason _reason;
    std::string _what;
};

/// Thread safe implementation of a queue. The template parameters indicate the
/// type of the queued messages, the max size of the queue, and whether to
/// overwrite old messages when the queue is full. This queue implementation
/// does not use dynamic memory, but instead uses a ring buffer, making it more
/// safe since the needed memory is known at compile time.
template <typename Type, size_t max_size = 1, bool overwrite = false>
class BelugaQueue {
  public:
    /**
     * @brief Put an item into the queue
     *
     * @param[in] item The item to put into the queue
     * @param[in] block If `true`, block until space is available, otherwise
     * throw an exception or overwrite the oldest item depending on the
     * configuration
     * @param[in] timeout_ms The timeout in milliseconds to wait for space to
     * become available
     * @throws BelugaQueueException If the queue is full
     */
    void put(Type &item, bool block = true,
             const std::chrono::milliseconds &timeout_ms =
                 std::chrono::milliseconds::zero());

    /**
     * @brief Get an item from the queue
     * @param[in] block If `true`, block until an item is available, otherwise
     * throw an exception
     * @param[in] timeout_ms The timeout in milliseconds to wait for an item to
     * become available
     * @return The oldest item in the queue
     * @throws BelugaQueueException If the queue is empty
     */
    Type get(bool block = true, const std::chrono::milliseconds &timeout_ms =
                                    std::chrono::milliseconds::zero());

    /**
     * The number of items in the queue
     * @return The number of items in the queue
     */
    size_t size();

    /**
     * Check if the queue is empty
     * @return `true` if the queue is empty, `false` otherwise
     */
    bool empty();

    /**
     * Check if the queue is full
     * @return `true` if the queue is full, `false` otherwise
     */
    bool full();

    /**
     * Clear the queue
     */
    void clear();

  private:
    Type _buffer[max_size];
    size_t _producer_index = 0;
    size_t _consumer_index = 0;
    std::atomic_size_t _size = 0;

    std::mutex _lock;
    std::condition_variable _not_empty;
    std::condition_variable _space_available;
};

template <typename Type, size_t max_size, bool overwrite>
void BelugaQueue<Type, max_size, overwrite>::put(
    Type &item, bool block, const std::chrono::milliseconds &timeout_ms) {
    std::unique_lock<std::mutex> lock(_lock);

    if (!block) {
        if (!_space_available.wait_for(lock, timeout_ms,
                                       [this]() { return !full(); })) {
            if (!overwrite) {
                throw BelugaQueueException(BelugaQueueException::QUEUE_FULL);
            }
        }
    } else {
        _space_available.wait(lock, [this]() { return !full(); });
    }

    _buffer[_producer_index] = item;
    _producer_index = (_producer_index + 1) % max_size;
    _size.fetch_add(1);
    if (_size >= max_size) {
        _size = max_size;
    }
    _not_empty.notify_one();
}

template <typename Type, size_t max_size, bool overwrite>
Type BelugaQueue<Type, max_size, overwrite>::get(
    bool block, const std::chrono::milliseconds &timeout_ms) {
    Type ret;
    std::unique_lock<std::mutex> lock(_lock);

    if (!block) {
        if (!_not_empty.wait_for(lock, timeout_ms,
                                 [this]() { return !empty(); })) {
            throw BelugaQueueException(BelugaQueueException::QUEUE_EMPTY);
        }
    } else {
        _not_empty.wait(lock, [this]() { return !empty(); });
    }

    ret = _buffer[_consumer_index];
    _consumer_index = (_consumer_index + 1) % max_size;
    _size.fetch_sub(1);
    _space_available.notify_one();
    return ret;
}

template <typename Type, size_t max_size, bool overwrite>
size_t BelugaQueue<Type, max_size, overwrite>::size() {
    size_t size = _size;
    return size;
}

template <typename Type, size_t max_size, bool overwrite>
bool BelugaQueue<Type, max_size, overwrite>::empty() {
    size_t size = _size;
    return size == 0;
}

template <typename Type, size_t max_size, bool overwrite>
bool BelugaQueue<Type, max_size, overwrite>::full() {
    size_t size = _size;
    return size == max_size;
}

template <typename Type, size_t max_size, bool overwrite>
void BelugaQueue<Type, max_size, overwrite>::clear() {
    std::lock_guard<std::mutex> lock(_lock);
    _size = 0;
    _consumer_index = 0;
    _producer_index = 0;
}
} // namespace BelugaSerial

#endif
