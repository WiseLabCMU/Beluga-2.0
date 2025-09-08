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

double BelugaNeighbor::range() const noexcept { return _data.range; }

int8_t BelugaNeighbor::rssi() const noexcept { return _data.rssi; }

int64_t BelugaNeighbor::time() const noexcept { return _data.time; }

uint32_t BelugaNeighbor::exchange() const noexcept { return _data.exchange; }

void BelugaNeighbor::update(const BelugaFrame::NeighborUpdate &neighbor) {
    _data.range = neighbor.RANGE;
    _data.rssi = neighbor.RSSI;
    _data.time = neighbor.TIMESTAMP;
    _data.exchange = neighbor.EXCHANGE;
    updated_ = true;
}

BelugaNeighborBase::~BelugaNeighborBase() = default;

BelugaNeighborBase::BelugaNeighborBase(
    const BelugaFrame::NeighborUpdate &neighbor) {
    _id = neighbor.ID;
}

uint16_t BelugaNeighborBase::id() const noexcept { return _id; }

bool BelugaNeighborBase::updated() const noexcept { return updated_; }

void BelugaNeighborBase::updated(bool update) { updated_ = update; }
} // namespace BelugaSerial
