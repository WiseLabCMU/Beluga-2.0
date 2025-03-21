/**
 * @file beluga_frame.hpp
 *
 * @brief Parser class for Beluga frames
 *
 * @date 1/20/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_FRAME_BELUGA_FRAME_HPP
#define BELUGA_FRAME_BELUGA_FRAME_HPP

#include <exception>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace BelugaSerial {

/**
 * Range event metadata
 */
struct RangeEvent {
    uint16_t ID = 0;   ///< ID of the node that got responded to
    uint32_t EXCHANGE; ///< The exchange ID of the initiator node
    int64_t TIMESTAMP; ///< Timestamp of the event
};

/// Exception class for Beluga frame errors
class BelugaFrameError : public std::exception {
  public:
    /**
     * Constructs a BelugaFrameError with a given error message
     * @param error
     */
    explicit BelugaFrameError(std::string error) : message(std::move(error)) {}

    /**
     * Error message indicating what went wrong
     * @return The error code
     */
    [[nodiscard]] const char *what() const noexcept override {
        return message.c_str();
    }

  private:
    std::string message;
};

/// Class for parsing Beluga frames
class BelugaFrame {
  public:
    /**
     * Default constructor
     */
    BelugaFrame();

    /**
     * Constructor that accepts a vector of bytes to parse
     * @param[in] str Vector of bytes
     */
    explicit BelugaFrame(const std::vector<uint8_t> &str);

    /**
     * Constructor that accepts an array of bytes and its length
     * @param[in] str The array of bytes
     * @param[in] len The number of bytes
     */
    explicit BelugaFrame(const char *str, size_t len);

    /**
     * Destructor
     */
    ~BelugaFrame();

    /// Enum for Beluga frame types
    enum BelugaFrameType {
        COMMAND_RESPONSE, ///< Response to a command
        NEIGHBOR_UPDATE,  ///< Range update for a neighbor or multiple neighbors
        RANGING_EVENT, ///< Range event indicating that the node responded to a
                       ///< ranging request
        NEIGHBOR_DROP, ///< Indicates that a neighbor has been dropped
        START_EVENT,   ///< Indicates that the node has booted
        NO_TYPE        ///< No frame type. Used for error handling
    };

    /// Structure for neighbor update metadata
    struct NeighborUpdate {
        uint16_t ID;  ///< ID of the neighbor node
        int8_t RSSI;  ///< Received Signal Strength Indicator for the BLE
        double RANGE; ///< The calculated distance between the connected node
                      ///< and the neighbor node
        int64_t TIMESTAMP; ///< Timestamp of the neighbor ranging update
        uint32_t EXCHANGE; ///< The exchange ID of the last ranging exchange
                           ///< with the neighbor
    };

    /// Structure for decoded frame data
    struct DecodedFrame {
        BelugaFrame::BelugaFrameType type; ///< Type of the frame
        std::variant<std::string, std::vector<BelugaFrame::NeighborUpdate>,
                     RangeEvent, uint32_t>
            payload; ///< Payload of the frame. This is dependent on the frame
                     ///< type. Types are:
                     ///< - `COMMAND_RESPONSE`: `std::string`
                     ///< - `NEIGHBOR_UPDATE`:
                     ///< `std::vector<BelugaFrame::NeighborUpdate>`
                     ///< - `RANGING_EVENT`: `RangeEvent`
                     ///< - `NEIGHBOR_DROP`: `uint32_t`
                     ///< - `START_EVENT`: `std::string`
                     /// - `NO_TYPE`: No payload
    };

    /**
     * Checks if a frame is present in the given C-style byte array
     * @param[in] bytearray The bytes to check for a present frame
     * @param[in] len The number of bytes in the array
     * @param[in] error_no_footer If `true`, continue searching for a frame if
     * the footer is incorrect or missing. If `false`, end search and return
     * error values
     * @return tuple<start index, length, bytes left before header>
     * @return tuple<-1, -1, -1> if no frame is present
     */
    static std::tuple<ssize_t, ssize_t, ssize_t>
    frame_present(const char *bytearray, size_t len,
                  bool error_no_footer = true);

    /**
     * Checks if a frame is present in the given a byte array
     * @param[in] bytearray The bytes to check for a frame
     * @param error_no_footer If `true`, continue searching for a frame if the
     * footer is incorrect or missing. If `false`, end search and return error
     * values
     * @return tuple<start index, length, bytes left before header>
     * @return tuple<-1, -1, -1> if no frame is present
     */
    static std::tuple<ssize_t, ssize_t, ssize_t>
    frame_present(const std::vector<uint8_t> &bytearray,
                  bool error_no_footer = true);

    /**
     * Parse a frame from raw data as a C-styled array
     * @param[in] serial_data The raw data that has a frame
     * @param[in] start_index The start index of the frame
     * @param[in] len The number of bytes in the raw data passed in
     *
     * @note Use `frame_present` to find the start index of the frame
     *
     * @throws BelugaFrameError if unable to parse the frame
     */
    void parse_frame(const char *serial_data, size_t start_index, size_t len);

    /**
     * Parse a frame from raw data
     * @param[in] serial_data The raw data that has a frame
     * @param[in] start_index The start index of the
     *
     * @note Use `frame_present` to find the start index of the frame
     *
     * @throws BelugaFrameError if unable to parse the frame
     */
    void parse_frame(const std::vector<uint8_t> &serial_data,
                     size_t start_index);

    /**
     * Get the parsed or "deserialized" frame data
     * @return The decoded frame
     *
     * @note `parse_frame` must be called before this function to get valid data
     */
    [[nodiscard]] BelugaFrame::DecodedFrame get_parsed_data() const;

  private:
    BelugaFrame::DecodedFrame parsed_data;
};
}; // namespace BelugaSerial

#endif // BELUGA_FRAME_BELUGA_FRAME_HPP
