/**
 * @file beluga_frame.hpp
 *
 * @brief
 *
 * @date 1/20/25
 *
 * @author tom
 */

#ifndef BELUGA_FRAME_BELUGA_FRAME_HPP
#define BELUGA_FRAME_BELUGA_FRAME_HPP

#include <exception>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace BelugaSerial {

struct RangeEvent {
    uint16_t ID = 0;
    uint32_t EXCHANGE;
    int64_t TIMESTAMP;
};

class BelugaFrameError : public std::exception {
  public:
    explicit BelugaFrameError(std::string error) : message(std::move(error)) {}

    [[nodiscard]] const char *what() const noexcept override {
        return message.c_str();
    }

  private:
    std::string message;
};

class BelugaFrame {
  public:
    BelugaFrame();
    explicit BelugaFrame(const std::vector<uint8_t> &str);
    explicit BelugaFrame(const char *str, size_t len);
    ~BelugaFrame();

    enum BelugaFrameType {
        COMMAND_RESPONSE,
        NEIGHBOR_UPDATE,
        RANGING_EVENT,
        NEIGHBOR_DROP,
        START_EVENT,
        NO_TYPE
    };

    struct NeighborUpdate {
        uint16_t ID;
        int8_t RSSI;
        double RANGE;
        int64_t TIMESTAMP;
        uint32_t EXCHANGE;
    };

    struct DecodedFrame {
        BelugaFrame::BelugaFrameType type;
        std::variant<std::string, std::vector<BelugaFrame::NeighborUpdate>,
                     RangeEvent, uint32_t>
            payload;
    };

    static std::tuple<ssize_t, ssize_t, ssize_t>
    frame_present(const char *bytearray, size_t len,
                  bool error_no_footer = true);
    static std::tuple<ssize_t, ssize_t, ssize_t>
    frame_present(const std::vector<uint8_t> &bytearray,
                  bool error_no_footer = true);

    void parse_frame(const char *serial_data, size_t start_index, size_t len);
    void parse_frame(const std::vector<uint8_t> &serial_data,
                     size_t start_index);

    [[nodiscard]] BelugaFrame::DecodedFrame get_parsed_data() const;

  private:
    BelugaFrame::DecodedFrame parsed_data;
};
}; // namespace BelugaSerial

#endif // BELUGA_FRAME_BELUGA_FRAME_HPP
