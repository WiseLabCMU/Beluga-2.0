/**
 * @file beluga_serial.hpp
 *
 * @brief
 *
 * @date 8/29/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_SERIAL_BELUGA_SERIAL_HPP
#define BELUGA_SERIAL_BELUGA_SERIAL_HPP

#include <beluga/beluga_serial_base.hpp>

namespace BelugaSerial {

template <typename NeighborImpl = BelugaNeighbor> struct NeighborCallbacks {
    static_assert(std::is_base_of<BelugaNeighborBase, NeighborImpl>::value,
                  "NeighborImpl must derive from BelugaNeighborBase");
    /**
     * Callback for neighbor updates
     * Default is nullptr
     * @note If this is left unset, the library will use a queue to store
     * updates.
     */
    std::function<void(const std::vector<NeighborImpl> &)> neighbor_update_cb =
        nullptr;

    /**
     * Callback for range updates
     * Default is nullptr
     * @note If this is left unset, the library will use a queue to store
     * updates.
     */
    std::function<void(const std::vector<NeighborImpl> &)> range_updates_cb =
        nullptr;
};

template <typename NeighborImpl = BelugaNeighbor>
class BelugaSerial : public BelugaSerialBase {
    static_assert(std::is_base_of<BelugaNeighborBase, NeighborImpl>::value,
                  "NeighborImpl must derive from BelugaNeighborBase");

  public:
    BelugaSerial() : BelugaSerialBase() {}
    explicit BelugaSerial(const BelugaSerialAttributes &attr,
                          NeighborCallbacks<NeighborImpl> &neighbor_callbacks)
        : BelugaSerialBase(attr) {
        _neighbor_cb = neighbor_callbacks.neighbor_update_cb;
        _range_cb = neighbor_callbacks.range_updates_cb;
    }

    /**
     * Retrieves the list of neighbors from the queue.
     * @param[in,out] list List of neighbors to populate
     * @return `true` if a neighbor update was available, `false` otherwise
     * @throws BelugaQueueException if there was an error retrieving the
     * neighbors and the error was not because the queue was empty
     * @note This will always return `false` if the neighbor updates callback is
     * set.
     */
    [[maybe_unused]] bool get_neighbors(std::vector<NeighborImpl> &list);

    /**
     * Retrieves the list of ranges from the queue.
     * @param[in,out] list List of ranges to populate
     * @throws BelugaQueueException if there was an error retrieving the ranges
     * and the error was not because the queue was empty
     * @note The list will always be empty if the range updates callback is set.
     */
    [[maybe_unused]] void get_ranges(std::vector<NeighborImpl> &list);

  private:
    BelugaNeighborList<NeighborImpl> _neighbors;

    BelugaQueue<std::vector<NeighborImpl>, 1, true> _neighbor_queue;
    std::function<void(const std::vector<NeighborImpl> &)> _neighbor_cb =
        nullptr;

    BelugaQueue<std::vector<NeighborImpl>, 1, true> _range_queue;
    std::function<void(const std::vector<NeighborImpl> &)> _range_cb = nullptr;

    void publish_neighbor_update_() override;
    void publish_range_update_() override;
    void update_neighbor_list_(
        const std::vector<BelugaFrame::NeighborUpdate> &updates) override;
    void remove_from_neighbor_list_(uint32_t id) override;
    void clear_neighbor_list_() override;
    void clear_queues_() override;
};

template <typename NeighborImpl>
void BelugaSerial<NeighborImpl>::clear_queues_() {
    _range_queue.clear();
    _neighbor_queue.clear();
}

template <typename NeighborImpl>
[[maybe_unused]] void
BelugaSerial<NeighborImpl>::get_ranges(std::vector<NeighborImpl> &list) {
    list.clear();
    try {
        list = _range_queue.get(false);
    } catch (const BelugaQueueException &exc) {
        if (exc.reason() != BelugaQueueException::QUEUE_EMPTY) {
            throw;
        }
    }
}

template <typename NeighborImpl>
[[maybe_unused]] bool
BelugaSerial<NeighborImpl>::get_neighbors(std::vector<NeighborImpl> &list) {
    bool update = true;
    list.clear();

    try {
        list = _neighbor_queue.get(false);
    } catch (const BelugaQueueException &exc) {
        if (exc.reason() == BelugaQueueException::QUEUE_EMPTY) {
            update = false;
        } else {
            throw;
        }
    }

    return update;
}

template <typename NeighborImpl>
void BelugaSerial<NeighborImpl>::publish_neighbor_update_() {
    if (_neighbors.neighbor_updates()) {
        std::vector<NeighborImpl> updates;
        _neighbors.get_neighbors(updates);
        if (_neighbor_cb != nullptr) {
            _neighbor_cb(updates);
        } else {
            _neighbor_queue.put(updates, false);
        }
    }
}

template <typename NeighborImpl>
void BelugaSerial<NeighborImpl>::publish_range_update_() {
    if (_neighbors.range_updates()) {
        std::vector<NeighborImpl> updates;
        _neighbors.get_updates(updates);
        if (_range_cb != nullptr) {
            _range_cb(updates);
        } else {
            _range_queue.put(updates, false);
        }
    }
}

template <typename NeighborImpl>
void BelugaSerial<NeighborImpl>::update_neighbor_list_(
    const std::vector<BelugaFrame::NeighborUpdate> &updates) {
    _neighbors.update(updates);
}

template <typename NeighborImpl>
void BelugaSerial<NeighborImpl>::remove_from_neighbor_list_(uint32_t id) {
    _neighbors.remove(id);
}

template <typename NeighborImpl>
void BelugaSerial<NeighborImpl>::clear_neighbor_list_() {
    _neighbors.clear();
}
}; // namespace BelugaSerial

#endif // BELUGA_SERIAL_BELUGA_SERIAL_HPP
