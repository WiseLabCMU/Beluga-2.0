/**
 * @file beluga_frame.cpp
 *
 * @brief
 *
 * @date 1/20/25
 *
 * @author tom
 */

#include <beluga/beluga_frame.hpp>
#include <daw/json/daw_json_link.h>

#define BELUGA_MSG_HEADER          (uint8_t)'@'
#define BELUGA_MSG_FOOTER          (uint8_t)'*'

#define BELUGA_MSG_HEADER_OVERHEAD 1
#define BELUGA_MSG_LEN_OVERHEAD    2
#define BELUGA_MSG_TYPE_OVERHEAD   1
#define BELUGA_MSG_FOOTER_OVERHEAD 1

#define BELUGA_HEADER_OVERHEAD                                                 \
    (BELUGA_MSG_HEADER_OVERHEAD + BELUGA_MSG_LEN_OVERHEAD +                    \
     BELUGA_MSG_TYPE_OVERHEAD)
#define BELUGA_FRAME_OVERHEAD                                                  \
    (BELUGA_HEADER_OVERHEAD + BELUGA_MSG_FOOTER_OVERHEAD)

#define MSG_HEADER_OFFSET  0
#define MSG_LEN_OFFSET     (MSG_HEADER_OFFSET + BELUGA_MSG_HEADER_OVERHEAD)
#define MSG_TYPE_OFFSET    (MSG_LEN_OFFSET + BELUGA_MSG_LEN_OVERHEAD)
#define MSG_PAYLOAD_OFFSET (MSG_TYPE_OFFSET + BELUGA_MSG_TYPE_OVERHEAD)

#define CONSTRUCT_PAYLOAD_LEN(array_, i)                                       \
    ((uint16_t)(uint8_t)(array_)[i] |                                          \
     (uint16_t)((uint8_t)(array_)[(i) + 1] << __CHAR_BIT__))

namespace daw::json {
template <>
struct json_data_contract<BelugaSerial::BelugaFrame::NeighborUpdate> {
    using type = json_member_list<
        json_number<"ID", uint16_t>, json_number<"RSSI", int8_t>,
        json_number<"RANGE", double>, json_number<"TIMESTAMP", int64_t>,
        json_number<"EXCHANGE", uint32_t>>;
};

template <> struct json_data_contract<BelugaSerial::RangeEvent> {
    using type = json_member_list<json_number<"ID", uint16_t>,
                                  json_number<"EXCHANGE", uint32_t>,
                                  json_number<"TIMESTAMP", int64_t>>;
};
} // namespace daw::json

BelugaSerial::BelugaFrame::BelugaFrame() { parsed_data.type = NO_TYPE; }

BelugaSerial::BelugaFrame::BelugaFrame(const std::vector<uint8_t> &str) {
    auto [start_index, length, bytes_left] =
        BelugaSerial::BelugaFrame::frame_present(str);

    if (start_index >= 0) {
        parse_frame(str, start_index);
    } else {
        throw BelugaFrameError("Frame is not present");
    }
}

BelugaSerial::BelugaFrame::BelugaFrame(const char *str, size_t len) {
    auto [start_index, length, bytes_left] =
        BelugaSerial::BelugaFrame::frame_present(
            std::vector<uint8_t>((uint8_t *)str, (uint8_t *)str + len));

    if (start_index >= 0) {
        parse_frame(std::vector<uint8_t>(str, str + len), start_index);
    } else {
        throw BelugaFrameError("Frame is not present");
    }
}

void BelugaSerial::BelugaFrame::parse_frame(const char *serial_data,
                                            size_t start_index, size_t len) {
    this->parse_frame(std::vector<uint8_t>(serial_data, serial_data + len),
                      start_index);
}

void BelugaSerial::BelugaFrame::parse_frame(
    const std::vector<uint8_t> &serial_data, size_t start_index) {
    size_t payload_len =
        CONSTRUCT_PAYLOAD_LEN(serial_data, start_index + MSG_LEN_OFFSET);
    BelugaFrameType type =
        (BelugaFrameType)serial_data[start_index + MSG_TYPE_OFFSET];
    std::string _payload = std::string(
        serial_data.begin() + start_index + MSG_PAYLOAD_OFFSET,
        serial_data.begin() + start_index + MSG_PAYLOAD_OFFSET + payload_len);
    size_t last_index;

    try {
        switch (type) {
        case COMMAND_RESPONSE:
        case START_EVENT:
            this->parsed_data.payload = _payload;
            break;
        case NEIGHBOR_UPDATE:
            this->parsed_data.payload =
                daw::json::from_json_array<NeighborUpdate>(_payload);
            break;
        case RANGING_EVENT:
            this->parsed_data.payload =
                daw::json::from_json<BelugaSerial::RangeEvent>(_payload);
            break;
        case NEIGHBOR_DROP:
            this->parsed_data.payload = (uint32_t)stoull(_payload, &last_index);
            if (last_index != payload_len) {
                throw std::invalid_argument("");
            }
            break;
        default:
            type = NO_TYPE;
            break;
        }
    } catch (daw::json::json_exception const &jex) {
        throw BelugaFrameError(jex.reason());
    } catch (std::invalid_argument const &invalid) {
        throw BelugaFrameError(
            "Unable to convert integer due to it not being an integer");
    } catch (std::out_of_range const &range) {
        throw BelugaFrameError("Integer passed in is out of range");
    }

    this->parsed_data.type = type;
}

BelugaSerial::BelugaFrame::DecodedFrame
BelugaSerial::BelugaFrame::get_parsed_data() const {
    return this->parsed_data;
}

std::tuple<ssize_t, ssize_t, ssize_t>
BelugaSerial::BelugaFrame::frame_present(const char *bytearray, size_t len,
                                         bool error_no_footer) {
    return frame_present(
        std::vector<uint8_t>((uint8_t *)bytearray, (uint8_t *)bytearray + len),
        error_no_footer);
}

std::tuple<ssize_t, ssize_t, ssize_t>
BelugaSerial::BelugaFrame::frame_present(const std::vector<uint8_t> &bytearray,
                                         bool error_no_footer) {
    size_t len = bytearray.size();
    size_t header_index, type_index, payload_len_index, frame_size;
    size_t payload_len, footer_index;

    for (size_t i = 0; i < len; i++) {
        if (bytearray[i] != BELUGA_MSG_HEADER) {
            continue;
        }
        header_index = i;

        type_index = header_index + MSG_TYPE_OFFSET;
        if (type_index > len) {
            continue;
        }

        payload_len_index = header_index + MSG_LEN_OFFSET;
        payload_len = CONSTRUCT_PAYLOAD_LEN(bytearray, payload_len_index);

        footer_index = header_index + BELUGA_HEADER_OVERHEAD + payload_len;

        if (footer_index >= len && error_no_footer) {
            continue;
        }

        if (error_no_footer &&
            (uint8_t)bytearray[footer_index] != BELUGA_MSG_FOOTER) {
            continue;
        }

        frame_size = BELUGA_FRAME_OVERHEAD + payload_len;
        return {header_index, frame_size,
                footer_index - (len - header_index) + 1};
    }

    return {-1, -1, -1};
}

BelugaSerial::BelugaFrame::~BelugaFrame() = default;
