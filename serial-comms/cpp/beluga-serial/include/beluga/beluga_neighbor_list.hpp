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

/// Class representing a neighbor in the Beluga network
class BelugaNeighbor {
  public:
    /**
     * Default constructor
     */
    BelugaNeighbor() = default;

    /**
     * Constructs a BelugaNeighbor from a NeighborUpdate
     * @param[in] neighbor The NeighborUpdate to initialize the BelugaNeighbor
     */
    explicit BelugaNeighbor(const BelugaFrame::NeighborUpdate &neighbor);

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
    ~BelugaNeighbor() = default;

    /**
     * Copy assignment operator
     * @param[in] copy The BelugaNeighbor to copy
     * @return A reference to the copied BelugaNeighbor
     */
    BelugaNeighbor &operator=(const BelugaNeighbor &copy) = default;

    /**
     * Neighbor ID
     * @return The neighbor's ID
     */
    [[nodiscard]] uint16_t id() const noexcept;

    /**
     * Neighbor range
     * @return The neighbor's range
     */
    [[nodiscard]] double range() const noexcept;

    /**
     * Neighbor RSSI
     * @return The neighbor's RSSI
     */
    [[nodiscard]] int8_t rssi() const noexcept;

    /**
     * Timestamp of the last ranging update
     * @return The timestamp of the last ranging update
     */
    [[nodiscard]] int64_t time() const noexcept;

    /**
     * Exchange ID of the last ranging exchange
     * @return The exchange ID of the last ranging exchange
     */
    [[nodiscard]] uint32_t exchange() const noexcept;

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
    void update(const BelugaFrame::NeighborUpdate &neighbor);

  private:
    uint16_t _id = 0;
    double _range = 0.0;
    int8_t _rssi = 0;
    int64_t _time = 0;
    uint32_t _exchange = 0;
    bool _updated = false;
};

/// Class representing a list of Beluga neighbors
class BelugaNeighborList {
  public:
    /**
     * Default constructor
     */
    BelugaNeighborList() = default;

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
    void get_updates(std::vector<BelugaNeighbor> &updates);

    /**
     * Retrieves all neighbors from the list
     * @param[out] neighbors The list of all neighbors
     */
    void get_neighbors(std::vector<BelugaNeighbor> &neighbors);

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
    [[nodiscard]] bool neighbor_updates() const noexcept;

    /**
     * Checks if there are any range updates
     * @return `true` if there are range updates, `false` otherwise
     */
    [[nodiscard]] bool range_updates() const noexcept;

  private:
    std::map<uint16_t, BelugaNeighbor> _list;
    bool _neighbors_update = false;
    bool _range_update = false;
};
} // namespace BelugaSerial

#endif // BELUGA_SERIAL_BELUGA_NEIGHBOR_LIST_HPP
