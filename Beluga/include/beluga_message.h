/**
 * @file beluga_message.h
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

#ifndef BELUGA_BELUGA_MESSAGE_H
#define BELUGA_BELUGA_MESSAGE_H

#include <ble_app.h>
#include <stdint.h>

/**
 * Header byte for a Beluga frame
 */
#define BELUGA_MSG_HEADER (uint8_t)'@'

/**
 * Trailer byte for a Beluga frame
 */
#define BELUGA_MSG_FOOTER (uint8_t)'*'

/**
 * Number of bytes required for the header byte
 */
#define BELUGA_MSG_HEADER_OVERHEAD 1

/**
 * Number of bytes required by the payload length field
 */
#define BELUGA_MSG_LEN_OVERHEAD 2

/**
 * Number of bytes required for the length field
 */
#define BELUGA_MSG_TYPE_OVERHEAD 1

/**
 * Number of bytes required for the footer
 */
#define BELUGA_MSG_FOOTER_OVERHEAD 1

enum beluga_msg_type {
    COMMAND_RESPONSE, ///< Responding to command
    NEIGHBOR_UPDATES, ///< Sending the neighbor list
    RANGING_EVENT,    ///< Response to an initiator
    NEIGHBOR_DROP,    ///< Can no longer find neighbor in network
    START_EVENT,      ///< A boot event
};

/**
 * Structure for describing a ranging event
 */
struct ranging_event {
    uint32_t id;          ///< The initiator's ID
    uint32_t exchange_id; ///< The exchange ID (This is set by the initiator)
    int64_t timestamp;    ///< The time the ranging event was recorded
};

/**
 * Beluga message information
 */
struct beluga_msg {
    enum beluga_msg_type type;
    union {
        const char *response; ///< COMMAND_RESPONSE
        struct {
            const struct node *neighbor_list; ///< NEIGHBOR_UPDATE
            bool stream;                      ///< flag if in stream mode
        };
        const struct ranging_event *event; ///< RANGING_EVENT
        uint32_t dropped_neighbor;         ///< NEIGHBOR_DROP
        const char *node_version;          ///< START_EVENT
    } payload;
};

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
int construct_frame(const struct beluga_msg *msg, uint8_t buffer[], size_t len);

/**
 * @brief Calculates the frame length for the given beluga message
 * @param[in] msg The message to calculate the frame length for
 * @return Frame length
 * @return -EINVAL if msg or message payload is NULL
 * @return negative error code otherwise
 */
int frame_length(const struct beluga_msg *msg);

#endif // BELUGA_BELUGA_MESSAGE_H
