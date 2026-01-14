/**
 * @file beluga_neighbor_list.hpp
 *
 * @brief Manages a list of Beluga neighbors
 *
 * @date 1/30/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_SERIAL_BELUGA_NEIGHBOR_LIST_HPP
#define BELUGA_SERIAL_BELUGA_NEIGHBOR_LIST_HPP

#include <beluga/beluga_frame.hpp>
#include <exception>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace BelugaSerial {

/// Exception class for Beluga entry errors
class BelugaEntryError : std::exception {
  public:
    /**
     * Constructs a BelugaEntryError with a given error message
     * @param[in] msg The error message
     */
    explicit BelugaEntryError(const char *msg) { _msg = msg; }

    /**
     * Error message indicating what went wrong
     * @return The error message
     */
    [[nodiscard]] const char *what() const noexcept override {
        return _msg.c_str();
    }

  private:
    std::string _msg;
};

class BelugaNeighborBase {
  public:
    BelugaNeighborBase() = default;
    explicit BelugaNeighborBase(const BelugaFrame::NeighborUpdate &neighbor);
    BelugaNeighborBase(const BelugaNeighborBase &) = default;
    BelugaNeighborBase(BelugaNeighborBase &&) = default;
    virtual ~BelugaNeighborBase() = 0;

    BelugaNeighborBase &operator=(const BelugaNeighborBase &copy) = default;

    [[nodiscard]] uint16_t id() const noexcept;

    /**
     * Neighbor range
     * @return The neighbor's range
     */
    [[nodiscard]] virtual double range() const = 0;

    /**
     * Neighbor RSSI
     * @return The neighbor's RSSI
     */
    [[nodiscard]] virtual int8_t rssi() const = 0;

    /**
     * Timestamp of the last ranging update
     * @return The timestamp of the last ranging update
     */
    [[nodiscard]] virtual int64_t time() const = 0;

    /**
     * Exchange ID of the last ranging exchange
     * @return The exchange ID of the last ranging exchange
     */
    [[nodiscard]] virtual uint32_t exchange() const = 0;

    /**
     * Indicates if the neighbor has been updated since the last read
     * @return `true` if the neighbor has been updated, `false` otherwise
     */
    [[nodiscard]] bool updated() const noexcept;
    void updated(bool update);

    /**
     * Updates the neighbor's information with a new NeighborUpdate
     * @param[in] neighbor The NeighborUpdate to update the BelugaNeighbor
     */
    virtual void update(const BelugaFrame::NeighborUpdate &neighbor) = 0;

  protected:
    struct NeighborData {
        double range = 0.0;
        int8_t rssi = 0;
        int64_t time = 0;
        uint32_t exchange = 0;
    };

    bool updated_ = false;

  private:
    uint16_t _id = 0;
};

/// Class representing a neighbor in the Beluga network
class BelugaNeighbor : public BelugaNeighborBase {
  public:
    /**
     * Default constructor
     */
    BelugaNeighbor() : BelugaNeighborBase() {}

    /**
     * Constructs a BelugaNeighbor from a NeighborUpdate
     * @param[in] neighbor The NeighborUpdate to initialize the BelugaNeighbor
     */
    explicit BelugaNeighbor(const BelugaFrame::NeighborUpdate &neighbor)
        : BelugaNeighborBase(neighbor) {
        update(neighbor);
    }

    /**
     * Copy constructor
     * @param[in] copy The BelugaNeighbor to copy
     */
    BelugaNeighbor(const BelugaNeighbor &) = default;

    /**
     * Move constructor
     * @param[in] copy The BelugaNeighbor to move
     */
    BelugaNeighbor(BelugaNeighbor &&) = default;

    /**
     * Destructor
     */
    ~BelugaNeighbor() override = default;

    /**
     * Copy assignment operator
     * @param[in] copy The BelugaNeighbor to copy
     * @return A reference to the copied BelugaNeighbor
     */
    BelugaNeighbor &operator=(const BelugaNeighbor &copy) = default;

    /**
     * Neighbor range
     * @return The neighbor's range
     */
    [[nodiscard]] double range() const override;

    /**
     * Neighbor RSSI
     * @return The neighbor's RSSI
     */
    [[nodiscard]] int8_t rssi() const override;

    /**
     * Timestamp of the last ranging update
     * @return The timestamp of the last ranging update
     */
    [[nodiscard]] int64_t time() const override;

    /**
     * Exchange ID of the last ranging exchange
     * @return The exchange ID of the last ranging exchange
     */
    [[nodiscard]] uint32_t exchange() const override;

    /**
     * Updates the neighbor's information with a new NeighborUpdate
     * @param[in] neighbor The NeighborUpdate to update the BelugaNeighbor
     */
    void update(const BelugaFrame::NeighborUpdate &neighbor) override;

  private:
    NeighborData _data;
};

/// Class representing a list of Beluga neighbors
template <typename NeighborImpl = BelugaNeighbor> class BelugaNeighborList {
  public:
    /**
     * Default constructor
     */
    BelugaNeighborList() {
        static_assert(std::is_base_of<BelugaNeighborBase, NeighborImpl>::value,
                      "NeighborImpl must inherit from BelugaNeighborBase");
    }

    /**
     * Destructor
     */
    ~BelugaNeighborList() = default;

    /**
     * Updates the neighbor list with new NeighborUpdates
     * @param[in] updates The NeighborUpdates to add to the list
     */
    void update(const std::vector<BelugaFrame::NeighborUpdate> &updates);

    /**
     * Removes a neighbor from the list by ID
     * @param[in] node_id The ID of the neighbor to remove
     */
    void remove(uint32_t node_id);

    /**
     * Retrieves the updated neighbors from the list
     * @param[out] updates The updated neighbors
     */
    void get_updates(std::vector<NeighborImpl> &updates);

    /**
     * Retrieves all neighbors from the list
     * @param[out] neighbors The list of all neighbors
     */
    void get_neighbors(std::vector<NeighborImpl> &neighbors);

    /**
     * Retrieves the neighbor list without resetting the neighbor update flag.
     * @param neighbors The list of all the neighbors
     */
    void get_neighbors_no_update(std::vector<NeighborImpl> &neighbors);

    /**
     * Clears the neighbor list
     */
    void clear() noexcept;

    /**
     * Checks if there are any neighbor list updates
     * @return `true` if there are neighbor updates, `false` otherwise
     * @note This checks if a new neighbor has been added or an existing
     * neighbor has been removed
     */
    [[nodiscard]] bool neighbor_updates() noexcept;

    /**
     * Checks if there are any range updates
     * @return `true` if there are range updates, `false` otherwise
     */
    [[nodiscard]] bool range_updates() noexcept;

  private:
    std::mutex _lock;
    std::map<uint16_t, NeighborImpl> _list;
    bool _neighbors_update = false;
    bool _range_update = false;
};

template <typename NeighborImpl>
void BelugaNeighborList<NeighborImpl>::update(
    const std::vector<BelugaFrame::NeighborUpdate> &updates) {
    std::lock_guard<std::mutex> guard(_lock);
    for (auto neighbor : updates) {
        if (_list.find(neighbor.ID) == _list.end()) {
            _list[neighbor.ID] = NeighborImpl(neighbor);
            _range_update = true;
            _neighbors_update = true;
        } else {
            _list[neighbor.ID].update(neighbor);
            _range_update = true;
        }
    }
}

template <typename NeighborImpl>
void BelugaNeighborList<NeighborImpl>::remove(uint32_t node_id) {
    std::lock_guard<std::mutex> guard(_lock);
    if (_list.find((uint16_t)node_id) != _list.end()) {
        _list.erase((uint16_t)node_id);
        _neighbors_update = true;
    }
}

template <typename NeighborImpl>
void BelugaNeighborList<NeighborImpl>::get_updates(
    std::vector<NeighborImpl> &updates) {
    updates.clear();
    std::lock_guard<std::mutex> guard(_lock);
    for (auto &[_, value] : _list) {
        if (value.updated()) {
            updates.emplace_back(value);
            value.updated(false);
        }
    }
    _range_update = false;
}

template <typename NeighborImpl>
void BelugaNeighborList<NeighborImpl>::get_neighbors(
    std::vector<NeighborImpl> &neighbors) {
    std::lock_guard<std::mutex> guard(_lock);
    for (auto &[_, value] : _list) {
        neighbors.emplace_back(value);
    }
    _neighbors_update = false;
}

template <typename NeighborImpl>
void BelugaNeighborList<NeighborImpl>::get_neighbors_no_update(
    std::vector<NeighborImpl> &neighbors) {
    std::lock_guard<std::mutex> guard(_lock);
    for (auto &[_, value] : _list) {
        neighbors.emplace_back(value);
    }
}

template <typename NeighborImpl>
void BelugaNeighborList<NeighborImpl>::clear() noexcept {
    std::lock_guard<std::mutex> guard(_lock);
    if (!_list.empty()) {
        _list.clear();
        _neighbors_update = true;
        _range_update = false;
    }
}

template <typename NeighborImpl>
bool BelugaNeighborList<NeighborImpl>::neighbor_updates() noexcept {
    std::lock_guard<std::mutex> guard(_lock);
    return _neighbors_update;
}

template <typename NeighborImpl>
bool BelugaNeighborList<NeighborImpl>::range_updates() noexcept {
    std::lock_guard<std::mutex> guard(_lock);
    return _range_update;
}
} // namespace BelugaSerial

#endif // BELUGA_SERIAL_BELUGA_NEIGHBOR_LIST_HPP
