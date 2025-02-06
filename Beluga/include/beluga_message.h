/**
 * @file beluga_message.h
 *
 * @brief
 *
 * @date 1/10/25
 *
 * @author tom
 */

#ifndef BELUGA_BELUGA_MESSAGE_H
#define BELUGA_BELUGA_MESSAGE_H

#include <ble_app.h>
#include <stdint.h>

#define BELUGA_MSG_HEADER          (uint8_t)'@'
#define BELUGA_MSG_FOOTER          (uint8_t)'*'

#define BELUGA_MSG_HEADER_OVERHEAD 1
#define BELUGA_MSG_LEN_OVERHEAD    2
#define BELUGA_MSG_TYPE_OVERHEAD   1
#define BELUGA_MSG_FOOTER_OVERHEAD 1

enum beluga_msg_type {
    COMMAND_RESPONSE,
    NEIGHBOR_UPDATES,
    RANGING_EVENT,
    NEIGHBOR_DROP,
    START_EVENT,
};

struct ranging_event {
    uint32_t id;
    uint32_t exchange_id;
    int64_t timestamp;
};

struct beluga_msg {
    enum beluga_msg_type type;
    union {
        const char *response; ///< COMMAND_RESPONSE
        struct {
            const struct node *neighbor_list; ///< NEIGHBOR_UPDATE
            bool stream;
        };
        const struct ranging_event *event; ///< RANGING_EVENT
        uint32_t dropped_neighbor;         ///< NEIGHBOR_DROP
        const char *node_version;          ///< START_EVENT
    } payload;
};

#if defined(CONFIG_BELUGA_FRAMES)
int construct_frame(const struct beluga_msg *msg, uint8_t buffer[], size_t len);
int frame_length(const struct beluga_msg *msg);
#else
#define construct_frame(...) -ENOTSUP
#define frame_length(...)    -ENOTSUP
#endif // defined(CONFIG_BELUGA_FRAMES)

#endif // BELUGA_BELUGA_MESSAGE_H
