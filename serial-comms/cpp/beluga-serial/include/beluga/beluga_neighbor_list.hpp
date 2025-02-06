/**
 * @file beluga_neighbor_list.hpp
 *
 * @brief
 *
 * @date 1/30/25
 *
 * @author tom
 */

#ifndef BELUGA_SERIAL_BELUGA_NEIGHBOR_LIST_HPP
#define BELUGA_SERIAL_BELUGA_NEIGHBOR_LIST_HPP

#include <beluga/beluga_frame.hpp>
#include <exception>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace BelugaSerial {
class BelugaEntryError : std::exception {
  public:
    explicit BelugaEntryError(const char *msg) { _msg = msg; }
    [[nodiscard]] const char *what() const noexcept override {
        return _msg.c_str();
    }

  private:
    std::string _msg;
};

class BelugaNeighbor {
  public:
    BelugaNeighbor() = default;
    explicit BelugaNeighbor(const BelugaFrame::NeighborUpdate &neighbor);
    BelugaNeighbor(const BelugaNeighbor &) = default;
    BelugaNeighbor(BelugaNeighbor &&) = default;
    ~BelugaNeighbor() = default;

    BelugaNeighbor &operator=(const BelugaNeighbor &copy) = default;

    [[nodiscard]] uint16_t id() const noexcept;
    [[nodiscard]] double range() const noexcept;
    [[nodiscard]] int8_t rssi() const noexcept;
    [[nodiscard]] int64_t time() const noexcept;
    [[nodiscard]] uint32_t exchange() const noexcept;

    [[nodiscard]] bool updated() const noexcept;
    void updated(bool update);

    void update(const BelugaFrame::NeighborUpdate &neighbor);

  private:
    uint16_t _id = 0;
    double _range = 0.0;
    int8_t _rssi = 0;
    int64_t _time = 0;
    uint32_t _exchange = 0;
    bool _updated = false;
};

class BelugaNeighborList {
  public:
    BelugaNeighborList() = default;
    ~BelugaNeighborList() = default;

    void update(const std::vector<BelugaFrame::NeighborUpdate> &updates);
    void remove(uint32_t node_id);
    void get_updates(std::vector<BelugaNeighbor> &updates);
    void get_neighbors(std::vector<BelugaNeighbor> &neighbors);
    void clear() noexcept;

    [[nodiscard]] bool neighbor_updates() const noexcept;
    [[nodiscard]] bool range_updates() const noexcept;

  private:
    std::map<uint16_t, BelugaNeighbor> _list;
    bool _neighbors_update = false;
    bool _range_update = false;
};
} // namespace BelugaSerial

#endif // BELUGA_SERIAL_BELUGA_NEIGHBOR_LIST_HPP
