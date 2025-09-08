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
template <typename NeighborList = BelugaNeighborList>
class BelugaSerial : public BelugaSerialBase {
    static_assert(std::is_base_of<BelugaNeighborListBase, NeighborList>::value,
                  "NeighborList must derive from BelugaNeighborListBase");

  public:
    BelugaSerial() : BelugaSerialBase() {}
    explicit BelugaSerial(const BelugaSerialAttributes &attr)
        : BelugaSerialBase(attr) {}

  private:
    NeighborList _neighbors;

    void publish_neighbor_update_() override;
    void publish_range_update_() override;
    void update_neighbor_list_(
        const std::vector<BelugaFrame::NeighborUpdate> &updates) override;
    void remove_from_neighbor_list_(uint32_t id) override;
    void clear_neighbor_list_() override;
};

template <typename NeighborList>
void BelugaSerial<NeighborList>::publish_neighbor_update_() {
    if (_neighbors.neighbor_updates()) {
        std::vector<BelugaNeighbor> updates;
        _neighbors.get_neighbors(updates);
        _publish_neighbor_updates_(updates);
    }
}

template <typename NeighborList>
void BelugaSerial<NeighborList>::publish_range_update_() {
    if (_neighbors.range_updates()) {
        std::vector<BelugaNeighbor> updates;
        _neighbors.get_updates(updates);
        _publish_range_updates_(updates);
    }
}

template <typename NeighborList>
void BelugaSerial<NeighborList>::update_neighbor_list_(
    const std::vector<BelugaFrame::NeighborUpdate> &updates) {
    _neighbors.update(updates);
}

template <typename NeighborList>
void BelugaSerial<NeighborList>::remove_from_neighbor_list_(uint32_t id) {
    _neighbors.remove(id);
}

template <typename NeighborList>
void BelugaSerial<NeighborList>::clear_neighbor_list_() {
    _neighbors.clear();
}
}; // namespace BelugaSerial

#endif // BELUGA_SERIAL_BELUGA_SERIAL_HPP
