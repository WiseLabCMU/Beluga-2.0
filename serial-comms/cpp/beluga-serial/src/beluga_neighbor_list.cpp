/**
 * @file beluga_neighbor_list.cpp
 *
 * @brief
 *
 * @date 1/30/25
 *
 * @author tom
 */

#include <beluga/beluga_frame.hpp>
#include <beluga/beluga_neighbor_list.hpp>

namespace BelugaSerial {
BelugaNeighbor::BelugaNeighbor(const BelugaFrame::NeighborUpdate &neighbor) {
    _id = neighbor.ID;
    update(neighbor);
}

uint16_t BelugaNeighbor::id() const noexcept { return _id; }

double BelugaNeighbor::range() const noexcept { return _range; }

int8_t BelugaNeighbor::rssi() const noexcept { return _rssi; }

int64_t BelugaNeighbor::time() const noexcept { return _time; }

uint32_t BelugaNeighbor::exchange() const noexcept { return _exchange; }

bool BelugaNeighbor::updated() const noexcept { return _updated; }

void BelugaNeighbor::updated(bool update) { _updated = update; }

void BelugaNeighbor::update(const BelugaFrame::NeighborUpdate &neighbor) {
    _range = neighbor.RANGE;
    _rssi = neighbor.RSSI;
    _time = neighbor.TIMESTAMP;
    _exchange = neighbor.EXCHANGE;
    _updated = true;
}

void BelugaNeighborList::update(
    const std::vector<BelugaFrame::NeighborUpdate> &updates) {
    for (auto neighbor : updates) {
        if (_list.find(neighbor.ID) == _list.end()) {
            _list[neighbor.ID] = BelugaNeighbor(neighbor);
            _range_update = true;
            _neighbors_update = true;
        } else {
            _list[neighbor.ID].update(neighbor);
            _range_update = true;
        }
    }
}

void BelugaNeighborList::remove(uint32_t node_id) {
    if (_list.find((uint16_t)node_id) != _list.end()) {
        _list.erase((uint16_t)node_id);
        _neighbors_update = true;
    }
}

void BelugaNeighborList::get_updates(std::vector<BelugaNeighbor> &updates) {
    updates.clear();
    for (auto &[_, value] : _list) {
        if (value.updated()) {
            updates.emplace_back(value);
            value.updated(false);
        }
    }
    _range_update = false;
}

void BelugaNeighborList::get_neighbors(std::vector<BelugaNeighbor> &neighbors) {
    for (auto &[_, value] : _list) {
        neighbors.emplace_back(value);
    }
    _neighbors_update = false;
}

void BelugaNeighborList::clear() noexcept {
    if (!_list.empty()) {
        _list.clear();
        _neighbors_update = true;
        _range_update = false;
    }
}

bool BelugaNeighborList::neighbor_updates() const noexcept {
    return _neighbors_update;
}

bool BelugaNeighborList::range_updates() const noexcept {
    return _range_update;
}
} // namespace BelugaSerial
