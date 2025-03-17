/**
 * @file beluga_message.c
 *
 * @brief Wraps payload data into a frame. The frame indicates what data
 * is being transmitted and how much data is being transmitted to make
 * processing easier with a software application.
 *
 * | Header | Payload Length | Type | Payload | Trailer |
 * | :----: | :------------: | :--: | :-----: | :-----: |
 * | 1 byte | 2 bytes        | 1 byte | 0-65535 bytes | 1 byte |
 * | @      | Varies         | 0-4    | Varies | * |
 *
 * @date 1/10/25
 *
 * @author tom
 */

#include <beluga_message.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/data/json.h>
#include <zephyr/logging/log.h>

/**
 * Logger for the message module
 */
LOG_MODULE_REGISTER(beluga_message_logger, CONFIG_BELUGA_MESSAGE_LOG_LEVEL);

/**
 * Frame header indicator offset
 */
#define MSG_HEADER_OFFSET 0

/**
 * Message length offset
 */
#define MSG_LEN_OFFSET (MSG_HEADER_OFFSET + BELUGA_MSG_HEADER_OVERHEAD)

/**
 * Message type offset
 */
#define MSG_TYPE_OFFSET (MSG_LEN_OFFSET + BELUGA_MSG_LEN_OVERHEAD)

/**
 * Message payload offset
 */
#define MSG_PAYLOAD_OFFSET (MSG_TYPE_OFFSET + BELUGA_MSG_TYPE_OVERHEAD)

/**
 * Total overhead of the beluga message header
 */
#define MSG_HEAD_OVERHEAD                                                      \
    (BELUGA_MSG_HEADER_OVERHEAD + BELUGA_MSG_LEN_OVERHEAD +                    \
     BELUGA_MSG_TYPE_OVERHEAD)

/**
 * Total overhead of the beluga message frame
 */
#define MSG_OVERHEAD (MSG_HEAD_OVERHEAD + BELUGA_MSG_FOOTER_OVERHEAD)

/**
 * Macro for calculating the message footer offset
 *
 * @param[in] payload_len The length of the payload
 */
#define MSG_FOOTER_OFFSET(payload_len) (MSG_HEAD_OVERHEAD + (payload_len))

/**
 * Helper macro that constructs a frame by setting the appropriate bytes
 * for the header, type, payload, and footer
 *
 * @param[in] _buf The buffer that stores the frame
 * @param[in] _payload_len The length of the payload
 * @param[in] _type The type of the frame
 */
#define ENCODE_FRAME(_buf, _payload_len, _type)                                \
    do {                                                                       \
        (_buf)[MSG_HEADER_OFFSET] = BELUGA_MSG_HEADER;                         \
        (_buf)[MSG_TYPE_OFFSET] = (uint8_t)(_type);                            \
        memcpy((_buf) + MSG_LEN_OFFSET, &(_payload_len), sizeof(uint16_t));    \
        (_buf)[MSG_FOOTER_OFFSET(_payload_len)] = BELUGA_MSG_FOOTER;           \
    } while (0)

/**
 * JSON payload data for encoding data
 *
 * @note The JSON uses 32-bit and 64-bit integers. Cannot
 * use integers that are less than 32 bits.
 */
struct node_json_struct {
    int32_t UUID; ///< Neighbor node ID
    int32_t RSSI; ///< RSSI of the node
#if defined(CONFIG_UWB_LOGIC_CLK)
    int32_t EXCHANGE;   ///< The exchange ID
#endif                  // defined(CONFIG_UWB_LOGIC_CLK)
    int64_t TIMESTAMP;  ///< Timestamp of last successful range measurement
    char str_RANGE[32]; ///< String representation of the range
    struct json_obj_token RANGE; ///< The JSON object token for the range
};

/**
 * Converts a float into a JSON token
 *
 * @param[in] json_obj The JSON object
 * @param[in] float_container The string that the float gets copied to.
 * Must be prefixed with "str_"
 * @param[in] float_prim The float struct member
 */
#define COPY_FLOAT(json_obj, float_container, float_prim)                      \
    do {                                                                       \
        (json_obj).float_container.length =                                    \
            snprintf((json_obj).str_##float_container,                         \
                     sizeof((json_obj).str_##float_container) - 1, "%f",       \
                     (double)(float_prim));                                    \
        (json_obj).float_container.start = (json_obj).str_##float_container;   \
    } while (0)

/**
 * Copies neighbor data into the node JSON struct
 *
 * @param[in] json_node The JSON struct being copied to
 * @param[in] node The neighbor node being copied from
 */
#define COPY_NODE(json_node, node)                                             \
    do {                                                                       \
        (json_node).UUID = (int32_t)(node).UUID;                               \
        (json_node).RSSI = (int32_t)(node).RSSI;                               \
        (json_node).TIMESTAMP = (node).time_stamp;                             \
        COPY_FLOAT(json_node, RANGE, (node).range);                            \
        (json_node).EXCHANGE = (int32_t)(node).exchange_id;                    \
    } while (0)

/**
 * JSON struct for the neighbor list. Supports up to the maximum
 * amount of neighbors.
 */
struct neighbor_list_json_struct {
    struct node_json_struct neighbors[MAX_ANCHOR_COUNT];
    size_t neighbors_len;
};

/**
 * Mapping JSON types to attributes of node_json_struct
 */
static const struct json_obj_descr neighbor_json[] = {
    JSON_OBJ_DESCR_PRIM_NAMED(struct node_json_struct, "ID", UUID,
                              JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct node_json_struct, RSSI, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct node_json_struct, TIMESTAMP, JSON_TOK_INT64),
    JSON_OBJ_DESCR_PRIM(struct node_json_struct, RANGE, JSON_TOK_FLOAT),
#if defined(CONFIG_UWB_LOGIC_CLK)
    JSON_OBJ_DESCR_PRIM(struct node_json_struct, EXCHANGE, JSON_TOK_NUMBER),
#endif // defined(CONFIG_UWB_LOGIC_CLK)
};

/**
 * Mapping JSON types to attributes of neighbor_list_json_struct
 */
static const struct json_obj_descr json_neighbor_list[] = {
    JSON_OBJ_DESCR_OBJ_ARRAY(struct neighbor_list_json_struct, neighbors,
                             MAX_ANCHOR_COUNT, neighbors_len, neighbor_json,
                             ARRAY_SIZE(neighbor_json)),
};

#if defined(CONFIG_UWB_LOGIC_CLK)
/**
 * Mapping JSON types to attributes of ranging_event
 */
static const struct json_obj_descr json_ranging_event[] = {
    JSON_OBJ_DESCR_PRIM_NAMED(struct ranging_event, "ID", id, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM_NAMED(struct ranging_event, "EXCHANGE", exchange_id,
                              JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM_NAMED(struct ranging_event, "TIMESTAMP", timestamp,
                              JSON_TOK_INT64),
};
#endif

/**
 * @brief Calculates the total frame size
 * @param[in] payload_size The payload size
 * @return The total frame size
 */
static int message_size(ssize_t payload_size) {
    if (payload_size == 0) {
        return MSG_OVERHEAD;
    }
    return MSG_OVERHEAD + payload_size;
}

/**
 * @brief Encodes a frame with the NEIGHBOR_UPDATE type. If no buffer is
 * provided, then only the calculated frame length would be returned.
 *
 * @param[in] msg The message to convert into a frame
 * @param[in] buffer The buffer to store the frame in.
 * @param[in] len The maximum length of the buffer
 * @return Frame length
 * @return -EINVAL if msg is NULL
 * @return -EAGAIN if neighbor list is empty
 * @return negative error code otherwise
 */
static ssize_t encode_neighbor_list(const struct beluga_msg *msg,
                                    uint8_t *buffer, size_t len) {
    struct neighbor_list_json_struct neighbors = {0};
    ssize_t numBytes = 0;
    int err;

    if (msg->payload.neighbor_list == NULL) {
        LOG_ERR("Invalid neighbor list");
        return -EINVAL;
    }

    for (size_t i = 0; i < MAX_ANCHOR_COUNT; i++) {
        if (msg->payload.neighbor_list[i].UUID != 0 &&
            (!msg->payload.stream ||
             msg->payload.neighbor_list[i].update_flag)) {

            COPY_NODE(neighbors.neighbors[neighbors.neighbors_len],
                      msg->payload.neighbor_list[i]);
            neighbors.neighbors_len++;
        }
    }

    if (neighbors.neighbors_len < 1) {
        LOG_ERR("Neighbor list empty");
        return -EAGAIN;
    }

    numBytes = json_calc_encoded_arr_len(json_neighbor_list, &neighbors);

    if (numBytes < 0 || buffer == NULL) {
        return numBytes;
    }

    err = json_arr_encode_buf(json_neighbor_list, &neighbors, buffer, len);

    if (err != 0) {
        LOG_ERR("Unable to encode JSON payload (%d)", err);
        return (ssize_t)err;
    }

    return numBytes;
}

#if defined(CONFIG_UWB_LOGIC_CLK)
/**
 * @brief Encodes a frame with the RANGING_EVENT type. If no buffer is
 * provided, then only the calculated frame length would be returned.
 *
 * @param[in] msg The message to convert into a frame
 * @param[in] buffer The buffer to store the frame in.
 * @param[in] len The maximum length of the buffer
 * @return Frame length
 * @return -EINVAL if msg is NULL
 * @return negative error code otherwise
 */
static ssize_t encode_ranging_event(const struct beluga_msg *msg,
                                    uint8_t *buffer, size_t len) {
    ssize_t numBytes = 0;
    int err;

    if (msg->payload.event == NULL) {
        LOG_ERR("Invalid event");
        return -EINVAL;
    }

    numBytes = json_calc_encoded_len(
        json_ranging_event, ARRAY_SIZE(json_ranging_event), msg->payload.event);

    if (numBytes < 0 || buffer == NULL) {
        return numBytes;
    }

    err =
        json_obj_encode_buf(json_ranging_event, ARRAY_SIZE(json_ranging_event),
                            msg->payload.event, buffer, len);

    if (err != 0) {
        LOG_ERR("Unable to encode JSON payload (%d)", err);
        return (ssize_t)err;
    }

    return numBytes;
}
#else
#define encode_ranging_event(...) = -ENOTSUP
#endif // CONFIG_UWB_LOGIC_CLK

/**
 * @brief Serializes a beluga message.
 *
 * @param[in] msg Pointer to the message to serialize
 * @param[in] buffer The buffer to place the serialized result into
 * @param[in] len The maximum length of the buffer
 * @return Frame size
 * @return -EINVAL if msg, buffer, or message payload is NULL
 * @return negative error code otherwise
 */
int construct_frame(const struct beluga_msg *msg, uint8_t buffer[],
                    size_t len) {
    ssize_t msgLen;
    int ret;
    if (msg == NULL || buffer == NULL) {
        LOG_ERR("Invalid intput parameters (%p) (%p)", msg, buffer);
        return -EINVAL;
    }

    switch (msg->type) {
    case COMMAND_RESPONSE: {
        if (msg->payload.response == NULL) {
            LOG_ERR("Invalid response");
            return -EINVAL;
        }
        msgLen = snprintf(buffer + MSG_PAYLOAD_OFFSET, len - MSG_OVERHEAD, "%s",
                          msg->payload.response);
        break;
    }
    case NEIGHBOR_UPDATES:
        msgLen = encode_neighbor_list(msg, buffer + MSG_PAYLOAD_OFFSET,
                                      len - MSG_OVERHEAD);
        break;
    case RANGING_EVENT:
        msgLen = encode_ranging_event(msg, buffer + MSG_PAYLOAD_OFFSET,
                                      len - MSG_OVERHEAD);
        break;
    case NEIGHBOR_DROP: {
        msgLen = snprintf(buffer + MSG_PAYLOAD_OFFSET, len - MSG_OVERHEAD,
                          "%" PRIu32, msg->payload.dropped_neighbor);
        break;
    }
    case START_EVENT: {
        if (msg->payload.node_version == NULL) {
            LOG_ERR("Invalid start event");
            return -EINVAL;
        }
        msgLen = snprintf(buffer + MSG_PAYLOAD_OFFSET, len - MSG_OVERHEAD, "%s",
                          msg->payload.node_version);
        break;
    }
    default:
        __ASSERT(false, "Invalid beluga message type: (%d)",
                 (uint32_t)msg->type);
    }

    if (msgLen >= 0) {
        ENCODE_FRAME(buffer, msgLen, msg->type);
    } else {
        return msgLen;
    }

    ret = message_size(msgLen);

    LOG_INF("Frame size: %d", ret);
    LOG_HEXDUMP_DBG(buffer, ret, "Frame: ");

    return ret;
}

/**
 * @brief Calculates the frame length for the given beluga message
 * @param[in] msg The message to calculate the frame length for
 * @return Frame length
 * @return -EINVAL if msg or message payload is NULL
 * @return negative error code otherwise
 */
int frame_length(const struct beluga_msg *msg) {
    ssize_t msgLen;

    if (msg == NULL) {
        LOG_ERR("Invalid message pointer");
        return -EINVAL;
    }

    switch (msg->type) {
    case COMMAND_RESPONSE: {
        if (msg->payload.response == NULL) {
            LOG_ERR("Invalid response");
            return -EINVAL;
        }
        msgLen = (ssize_t)strlen(msg->payload.response) + 1;
        break;
    }
    case NEIGHBOR_UPDATES:
        msgLen = encode_neighbor_list(msg, NULL, 0);
        break;
    case RANGING_EVENT:
        msgLen = encode_ranging_event(msg, NULL, 0);
        break;
    case NEIGHBOR_DROP: {
        msgLen = snprintf(NULL, 0, "%" PRIu32, msg->payload.dropped_neighbor);
        msgLen += 1;
        break;
    }
    case START_EVENT: {
        if (msg->payload.node_version == NULL) {
            LOG_ERR("Invalid start event");
            return -EINVAL;
        }
        msgLen = (ssize_t)strlen(msg->payload.node_version) + 1;
        break;
    }
    default:
        __ASSERT(false, "Invalid beluga message type: (%d)",
                 (uint32_t)msg->type);
    }

    if (msgLen < 0) {
        return msgLen;
    }

    return message_size(msgLen);
}
