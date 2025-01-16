/**
 * @file beluga_message.c
 *
 * @brief
 *
 * @date 1/10/25
 *
 * @author tom
 */

#if defined(CONFIG_BELUGA_FRAMES)
#include <beluga_message.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/data/json.h>

#define MSG_HEADER_OFFSET  0
#define MSG_LEN_OFFSET     (MSG_HEADER_OFFSET + BELUGA_MSG_HEADER_OVERHEAD)
#define MSG_TYPE_OFFSET    (MSG_LEN_OFFSET + BELUGA_MSG_LEN_OVERHEAD)
#define MSG_PAYLOAD_OFFSET (MSG_TYPE_OFFSET + BELUGA_MSG_TYPE_OVERHEAD)

#define MSG_HEAD_OVERHEAD                                                      \
    (BELUGA_MSG_HEADER_OVERHEAD + BELUGA_MSG_LEN_OVERHEAD +                    \
     BELUGA_MSG_TYPE_OVERHEAD)

#define MSG_OVERHEAD                   (MSG_HEAD_OVERHEAD + BELUGA_MSG_FOOTER_OVERHEAD)

#define MSG_FOOTER_OFFSET(payload_len) (MSG_HEAD_OVERHEAD + (payload_len))

#define ENCODE_FRAME(_buf, _payload_len, _type)                                \
    do {                                                                       \
        (_buf)[MSG_HEADER_OFFSET] = BELUGA_MSG_HEADER;                         \
        (_buf)[MSG_TYPE_OFFSET] = (uint8_t)(_type);                            \
        memcpy((_buf) + MSG_LEN_OFFSET, &(_payload_len), sizeof(uint16_t));    \
        (_buf)[MSG_FOOTER_OFFSET(_payload_len)] = BELUGA_MSG_FOOTER;           \
    } while (0)

struct node_json_struct {
    int32_t UUID;
    int32_t RSSI;
#if defined(CONFIG_UWB_LOGIC_CLK)
    int32_t EXCHANGE;
#endif // defined(CONFIG_UWB_LOGIC_CLK)
    int64_t TIMESTAMP;
    char str_RANGE[32];
    struct json_obj_token RANGE;
};

#define COPY_FLOAT(json_obj, float_container, float_prim)                      \
    do {                                                                       \
        (json_obj).float_container.length =                                    \
            snprintf((json_obj).str_##float_container,                         \
                     sizeof((json_obj).str_##float_container) - 1, "%f",       \
                     (double)(float_prim));                                    \
        (json_obj).float_container.start = (json_obj).str_##float_container;   \
    } while (0)

#define COPY_NODE(json_node, node)                                             \
    do {                                                                       \
        (json_node).UUID = (int32_t)(node).UUID;                               \
        (json_node).RSSI = (int32_t)(node).RSSI;                               \
        (json_node).TIMESTAMP = (node).time_stamp;                             \
        COPY_FLOAT(json_node, RANGE, (node).range);                            \
        (json_node).EXCHANGE = (int32_t)(node).exchange_id;                    \
    } while (0)

struct neighbor_list_json_struct {
    struct node_json_struct neighbors[MAX_ANCHOR_COUNT];
    size_t neighbors_len;
};

static const struct json_obj_descr neighbor_json[] = {
    JSON_OBJ_DESCR_PRIM(struct node_json_struct, UUID, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct node_json_struct, RSSI, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct node_json_struct, TIMESTAMP, JSON_TOK_INT64),
    JSON_OBJ_DESCR_PRIM(struct node_json_struct, RANGE, JSON_TOK_FLOAT),
#if defined(CONFIG_UWB_LOGIC_CLK)
    JSON_OBJ_DESCR_PRIM(struct node_json_struct, EXCHANGE, JSON_TOK_NUMBER),
#endif // defined(CONFIG_UWB_LOGIC_CLK)
};

static const struct json_obj_descr json_neighbor_list[] = {
    JSON_OBJ_DESCR_OBJ_ARRAY(struct neighbor_list_json_struct, neighbors,
                             MAX_ANCHOR_COUNT, neighbors_len, neighbor_json,
                             ARRAY_SIZE(neighbor_json)),
};

#if defined(CONFIG_UWB_LOGIC_CLK)
static const struct json_obj_descr json_ranging_event[] = {
    JSON_OBJ_DESCR_PRIM_NAMED(struct ranging_event, "ID", id, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM_NAMED(struct ranging_event, "EXCHANGE", exchange_id,
                              JSON_TOK_NUMBER),
};
#endif

static int message_size(ssize_t payload_size) {
    if (payload_size == 0) {
        return MSG_OVERHEAD;
    }
    return MSG_OVERHEAD + payload_size;
}

static ssize_t encode_neighbor_list(const struct beluga_msg *msg,
                                    uint8_t *buffer, size_t len) {
    struct neighbor_list_json_struct neighbors = {0};
    ssize_t numBytes = 0;
    int err;

    if (msg->payload.neighbor_list == NULL) {
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

    numBytes = json_calc_encoded_arr_len(json_neighbor_list, &neighbors);

    if (numBytes < 0 || buffer == NULL) {
        return numBytes;
    }

    err = json_arr_encode_buf(json_neighbor_list, &neighbors, buffer, len);

    if (err != 0) {
        return (ssize_t)err;
    }

    return numBytes;
}

#if defined(CONFIG_UWB_LOGIC_CLK)
static ssize_t encode_ranging_event(const struct beluga_msg *msg,
                                    uint8_t *buffer, size_t len) {
    ssize_t numBytes = 0;
    int err;

    if (msg->payload.event == NULL) {
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
        return (ssize_t)err;
    }

    return numBytes;
}
#else
#define encode_ranging_event(...) = -ENOTSUP
#endif // CONFIG_UWB_LOGIC_CLK

int construct_frame(const struct beluga_msg *msg, uint8_t buffer[],
                    size_t len) {
    ssize_t msgLen;
    if (msg == NULL || buffer == NULL) {
        return -EINVAL;
    }

    switch (msg->type) {
    case COMMAND_RESPONSE: {
        if (msg->payload.response == NULL) {
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
    case REBOOT_EVENT: {
        // No payload
        msgLen = 0;
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

    return message_size(msgLen);
}

int frame_length(const struct beluga_msg *msg) {
    ssize_t msgLen;

    if (msg == NULL) {
        return -EINVAL;
    }

    switch (msg->type) {
    case COMMAND_RESPONSE: {
        if (msg->payload.response == NULL) {
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
    default:
        __ASSERT(false, "Invalid beluga message type: (%d)",
                 (uint32_t)msg->type);
    }

    if (msgLen < 0) {
        return msgLen;
    }

    return message_size(msgLen);
}

#endif // defined(CONFIG_BELUGA_FRAMES)
