/**
 * @file main.cpp
 *
 * @brief
 *
 * @date 9/8/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <algorithm>
#include <beluga/beluga_neighbor_list.hpp>
#include <beluga/beluga_serial.hpp>
#include <iostream>
#include <ranges>
#include <vector>

constexpr unsigned int window_size = 3;
constexpr unsigned int median_idx = window_size / 2;
static_assert((window_size & 1u) == 1u, "Window size must be odd");

class BelugaNeighborMedianFilter final
    : public BelugaSerial::BelugaNeighborBase {
  public:
    BelugaNeighborMedianFilter() : BelugaSerial::BelugaNeighborBase() {
        _data.reserve(window_size);
    }
    explicit BelugaNeighborMedianFilter(
        const BelugaSerial::BelugaFrame::NeighborUpdate &neighbor)
        : BelugaSerial::BelugaNeighborBase(neighbor) {
        _data.reserve(window_size);
        update(neighbor);
    }

    ~BelugaNeighborMedianFilter() final = default;

    [[nodiscard]] double range() const final;
    [[nodiscard]] int8_t rssi() const final;
    [[nodiscard]] int64_t time() const final;
    [[nodiscard]] uint32_t exchange() const final;
    void
    update(const BelugaSerial::BelugaFrame::NeighborUpdate &neighbor) final;

  private:
    std::vector<NeighborData> _data;

    void
    _replace_oldest(const BelugaSerial::BelugaFrame::NeighborUpdate &neighbor);
    void
    _insert_entry(const BelugaSerial::BelugaFrame::NeighborUpdate &neighbor);
    void _sort();
};

double BelugaNeighborMedianFilter::range() const {
    if (_data.size() == window_size) {
        return _data[median_idx].range;
    } else if (!_data.empty()) {
        return _data[0].range;
    } else {
        throw std::runtime_error("Not sufficiently populated");
    }
}

int8_t BelugaNeighborMedianFilter::rssi() const {
    if (_data.size() == window_size) {
        return _data[median_idx].rssi;
    } else if (!_data.empty()) {
        return _data[0].rssi;
    } else {
        throw std::runtime_error("Not sufficiently populated");
    }
}

int64_t BelugaNeighborMedianFilter::time() const {
    if (_data.size() == window_size) {
        return _data[median_idx].time;
    } else if (!_data.empty()) {
        return _data[0].time;
    } else {
        throw std::runtime_error("Not sufficiently populated");
    }
}

uint32_t BelugaNeighborMedianFilter::exchange() const {
    if (_data.size() == window_size) {
        return _data[median_idx].exchange;
    } else if (!_data.empty()) {
        return _data[0].exchange;
    } else {
        throw std::runtime_error("Not sufficiently populated");
    }
}

void BelugaNeighborMedianFilter::update(
    const BelugaSerial::BelugaFrame::NeighborUpdate &neighbor) {
    if (_data.size() < window_size) {
        _insert_entry(neighbor);
        if (_data.size() == window_size) {
            updated_ = true;
        }
    } else {
        _replace_oldest(neighbor);
        _sort();
        updated_ = true;
    }
}

void BelugaNeighborMedianFilter::_replace_oldest(
    const BelugaSerial::BelugaFrame::NeighborUpdate &neighbor) {
    auto oldest = std::ranges::min_element(_data, {}, &NeighborData::time);
    oldest->time = neighbor.TIMESTAMP;
    oldest->exchange = neighbor.EXCHANGE;
    oldest->rssi = neighbor.RSSI;
    oldest->range = neighbor.RANGE;
}

void BelugaNeighborMedianFilter::_insert_entry(
    const BelugaSerial::BelugaFrame::NeighborUpdate &neighbor) {
    NeighborData data = {.range = neighbor.RANGE,
                         .rssi = neighbor.RSSI,
                         .time = neighbor.TIMESTAMP,
                         .exchange = neighbor.EXCHANGE};

    _data.emplace_back(data);
}

void BelugaNeighborMedianFilter::_sort() {
    std::ranges::sort(_data, {}, &NeighborData::range);
}

void range_update_cb(std::vector<BelugaNeighborMedianFilter> &updates) {
    for (const auto &update : updates) {
        std::cout << update.id() << "\n"
                  << update.range() << "\n"
                  << update.rssi() << "\n"
                  << update.time() << "\n"
                  << update.exchange() << "\n"
                  << "------\n";
    }
    std::cout << "\n";
}

int main() {
    BelugaSerial::BelugaSerialAttributes attr;
    BelugaSerial::NeighborCallbacks<BelugaNeighborMedianFilter> cb = {
        .range_updates_cb = [](auto updates) { range_update_cb(updates); },
    };

    BelugaSerial::BelugaSerial<BelugaNeighborMedianFilter> serial =
        BelugaSerial::BelugaSerial<BelugaNeighborMedianFilter>(attr, cb);

    serial.start();
    serial.start_ble();
    serial.start_uwb();

    while (true)
        ;

    return 0;
}
