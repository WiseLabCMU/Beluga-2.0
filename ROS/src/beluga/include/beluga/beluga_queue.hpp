/**
 * @file beluga_queue.hpp
 *
 * @brief
 *
 * @date 1/20/25
 *
 * @author tom
 */

#ifndef BELUGA_BELUGA_QUEUE_HPP
#define BELUGA_BELUGA_QUEUE_HPP

#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <exception>
#include <atomic>
#include <string>

namespace Beluga {
    class BelugaQueueException : public std::exception {
    public:
        enum QueueExceptionReason {
            QUEUE_EMPTY,
            QUEUE_FULL,
            QUEUE_TIMEOUT
        };
        explicit BelugaQueueException(const QueueExceptionReason &reason) : reason_(reason) {
            switch(reason) {
                case QUEUE_EMPTY: what_ =  "Queue Empty";
                    break;
                case QUEUE_FULL: what_ = "Queue Full";
                    break;
                case QUEUE_TIMEOUT: what_ = "Queue Timed Out";
                    break;
                default: what_ = "Unknown Queue Error";
                    break;
            }
        }

        [[nodiscard]] const char *what() const noexcept override {
            return what_.c_str();
        }

        [[nodiscard]] QueueExceptionReason reason() const noexcept {
            return reason_;
        }

    private:
        QueueExceptionReason reason_;
        std::string what_;
    };


    template<typename Type, size_t max_size = 1>
    class BelugaQueue {
    public:
        void put(Type &item, bool block = true, uint32_t timeout_ms = 0);
        Type get(bool block = true, uint32_t timeout_ms = 0);

        size_t size();
        bool empty();
        bool full();

    private:
        Type _buffer[max_size];
        size_t producer_index = 0;
        size_t consumer_index = 0;
        std::atomic_size_t _size = 0;

        std::mutex lock_;
        std::condition_variable not_empty_;
        std::condition_variable space_available_;
    };

    template<typename Type, size_t max_size>
    void BelugaQueue<Type, max_size>::put(Type &item, bool block, uint32_t timeout_ms) {
        std::unique_lock<std::mutex> lock(lock_);

        if (!block) {
            if (!space_available_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]() { return !full(); })) {
                throw BelugaQueueException(BelugaQueueException::QUEUE_FULL);
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

    template<typename Type, size_t max_size>
    Type BelugaQueue<Type, max_size>::get(bool block, uint32_t timeout_ms) {
        Type ret;
        std::unique_lock<std::mutex> lock(lock_);

        if (!block) {
            if (!not_empty_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]() { return !empty(); })) {
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

    template<typename Type, size_t max_size>
    size_t BelugaQueue<Type, max_size>::size() {
        size_t size_ = _size;
        return size_;
    }

    template<typename Type, size_t max_size>
    bool BelugaQueue<Type, max_size>::empty() {
        size_t size_ = _size;
        return size_ == 0;
    }

    template<typename Type, size_t max_size>
    bool BelugaQueue<Type, max_size>::full() {
        size_t size_ = _size;
        return size_ == max_size;
    }
}

#endif //BELUGA_BELUGA_QUEUE_HPP
