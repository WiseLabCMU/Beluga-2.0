/**
 * @file beluga_queue.hpp
 *
 * @brief Thread safe implementation of a queue
 *
 * @date 1/30/25
 *
 * @authro Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */
#ifndef _BELUGA_QUEUE
#define _BELUGA_QUEUE

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
        : reason_(reason) {
        switch (reason) {
        case QUEUE_EMPTY:
            what_ = "Queue Empty";
            break;
        case QUEUE_FULL:
            what_ = "Queue Full";
            break;
        case QUEUE_TIMEOUT:
            what_ = "Queue Timed Out";
            break;
        default:
            what_ = "Unknown Queue Error";
            break;
        }
    }

    /**
     * The reason for the exception as a string
     * @return The error reason
     */
    [[nodiscard]] const char *what() const noexcept override {
        return what_.c_str();
    }

    /**
     * @brief Get the reason for the exception
     * @return The reason for the exception
     */
    [[nodiscard]] QueueExceptionReason reason() const noexcept {
        return reason_;
    }

  private:
    QueueExceptionReason reason_;
    std::string what_;
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
    size_t producer_index = 0;
    size_t consumer_index = 0;
    std::atomic_size_t _size = 0;

    std::mutex lock_;
    std::condition_variable not_empty_;
    std::condition_variable space_available_;
};

template <typename Type, size_t max_size, bool overwrite>
void BelugaQueue<Type, max_size, overwrite>::put(
    Type &item, bool block, const std::chrono::milliseconds &timeout_ms) {
    std::unique_lock<std::mutex> lock(lock_);

    if (!block) {
        if (!space_available_.wait_for(lock, timeout_ms,
                                       [this]() { return !full(); })) {
            if (!overwrite) {
                throw BelugaQueueException(BelugaQueueException::QUEUE_FULL);
            }
        }
    } else {
        space_available_.wait(lock, [this]() { return !full(); });
    }

    _buffer[producer_index] = item;
    producer_index = (producer_index + 1) % max_size;
    _size.fetch_add(1);
    if (_size >= max_size) {
        _size = max_size;
    }
    not_empty_.notify_one();
}

template <typename Type, size_t max_size, bool overwrite>
Type BelugaQueue<Type, max_size, overwrite>::get(
    bool block, const std::chrono::milliseconds &timeout_ms) {
    Type ret;
    std::unique_lock<std::mutex> lock(lock_);

    if (!block) {
        if (!not_empty_.wait_for(lock, timeout_ms,
                                 [this]() { return !empty(); })) {
            throw BelugaQueueException(BelugaQueueException::QUEUE_EMPTY);
        }
    } else {
        not_empty_.wait(lock, [this]() { return !empty(); });
    }

    ret = _buffer[consumer_index];
    consumer_index = (consumer_index + 1) % max_size;
    _size.fetch_sub(1);
    space_available_.notify_one();
    return ret;
}

template <typename Type, size_t max_size, bool overwrite>
size_t BelugaQueue<Type, max_size, overwrite>::size() {
    size_t size_ = _size;
    return size_;
}

template <typename Type, size_t max_size, bool overwrite>
bool BelugaQueue<Type, max_size, overwrite>::empty() {
    size_t size_ = _size;
    return size_ == 0;
}

template <typename Type, size_t max_size, bool overwrite>
bool BelugaQueue<Type, max_size, overwrite>::full() {
    size_t size_ = _size;
    return size_ == max_size;
}

template <typename Type, size_t max_size, bool overwrite>
void BelugaQueue<Type, max_size, overwrite>::clear() {
    std::lock_guard<std::mutex> lock(lock_);
    _size = 0;
    consumer_index = 0;
    producer_index = 0;
}
} // namespace BelugaSerial

#endif
