/**
 * @file beluga_serial.hpp
 *
 * @brief
 *
 * @date 8/29/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_DTS_BELUGA_SERIAL_HPP
#define BELUGA_DTS_BELUGA_SERIAL_HPP

#include <beluga/beluga_serial_base.hpp>

namespace BelugaSerial {
template <typename NeighborList = BelugaNeighborList>
class BelugaSerial : public BelugaSerialBase {
    static_assert(std::is_base_of<BelugaNeighborListBase, NeighborList>::value,
                  "NeighborList must derive from BelugaNeighborListBase");

  public:
    explicit BelugaSerial(const BelugaSerialAttributes &attr)
        : BelugaSerialBase(attr) {}

  private:
    NeighborList _neighbors;

    void _publish_neighbor_update() override;
    void _publish_range_update() override;
    void _update_neighbor_list(
        const std::vector<BelugaFrame::NeighborUpdate> &updates) override;
    void _remove_from_neighbor_list(uint32_t id) override;
    void _clear_neighbor_list() override;
};

template <typename NeighborList>
void BelugaSerial<NeighborList>::_publish_neighbor_update() {
    if (_neighbors.neighbor_updates()) {
        std::vector<BelugaNeighbor> updates;
        _neighbors.get_neighbors(updates);
        if (_neighbor_cb != nullptr) {
            _neighbor_cb(updates);
        } else {
            _neighbor_queue.put(updates, false);
        }
    }
}

template <typename NeighborList>
void BelugaSerial<NeighborList>::_publish_range_update() {
    if (_neighbors.range_updates()) {
        std::vector<BelugaNeighbor> updates;
        _neighbors.get_updates(updates);
        if (_range_cb != nullptr) {
            _range_cb(updates);
        } else {
            _range_queue.put(updates, false);
        }
    }
}

template <typename NeighborList>
void BelugaSerial<NeighborList>::_update_neighbor_list(
    const std::vector<BelugaFrame::NeighborUpdate> &updates) {
    _neighbors.update(updates);
}

template <typename NeighborList>
void BelugaSerial<NeighborList>::_remove_from_neighbor_list(uint32_t id) {
    _neighbors.remove(id);
}

template <typename NeighborList>
void BelugaSerial<NeighborList>::_clear_neighbor_list() {
    _neighbors.clear();
}
}; // namespace BelugaSerial

#endif // BELUGA_DTS_BELUGA_SERIAL_HPP
