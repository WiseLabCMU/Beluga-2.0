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

#include <ble/ble_app.h>
#include <deca_device_api.h>
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

/**
 * Beluga message types
 */
enum beluga_msg_type {
    COMMAND_RESPONSE, ///< Responding to command
    NEIGHBOR_UPDATES, ///< Sending the neighbor list
    RANGING_EVENT,    ///< Response to an initiator
    NEIGHBOR_DROP,    ///< Can no longer find neighbor in network
    START_EVENT,      ///< A boot event
    UWB_RANGING_DROP, ///< Report that ranging failed
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
 * Structure for describing UWB event counts in 32-bit integers.
 */
struct uwb_counts {
    uint32_t PHE;   ///< Number of received header errors.
    uint32_t RSL;   ///< Number of received frame sync loss events.
    uint32_t CRCG;  ///< Number of good CRC received frames.
    uint32_t CRCB;  ///< Number of bad CRC (CRC error) received frames.
    uint32_t ARFE;  ///< Number of address filter errors.
    uint32_t OVER;  ///< Number of receiver overflows (used in
                    ///< double buffer mode).
    uint32_t SFDTO; ///< SFD timeouts.
    uint32_t PTO;   ///< Preamble timeouts.
    uint32_t RTO;   ///< RX frame wait timeouts.
    uint32_t TXF;   ///< Number of transmitted frames.
    uint32_t HPW;   ///< Half period warn.
    uint32_t TXW;   ///< Power up warn.
};

/**
 * Translates the dwt_deviceentcnts_t struct into a uwb_counts struct.
 * @param[in,out] dest The uwb_counts struct to copy the data into.
 * @param[in] src The dwt_deviceentcnts_t struct to copy the data from.
 */
static inline void copy_event_counts(struct uwb_counts *dest,
                                     const dwt_deviceentcnts_t *src) {
    // Need to do this because the deca api uses 16 bit ints instead of
    // 32 bit ints. The JSON library treats all number fields as 32 bit ints,
    // unless it is specified that it is a 64-bit int. This is true for ncs
    // v2.9.1, but it looks like it may have changed in v3.0.0. I refuse to
    // update to v3.0.0 because Nordic requires you to install their toolchain
    // through fucking VS code, and the toolchain installer through VS code
    // doesn't fucking work...
    if (dest == NULL || src == NULL) {
        return;
    }
    dest->PHE = src->PHE;
    dest->RSL = src->RSL;
    dest->CRCG = src->CRCG;
    dest->CRCB = src->CRCB;
    dest->ARFE = src->ARFE;
    dest->OVER = src->OVER;
    dest->SFDTO = src->SFDTO;
    dest->PTO = src->PTO;
    dest->RTO = src->RTO;
    dest->TXF = src->TXF;
    dest->HPW = src->HPW;
    dest->TXW = src->TXW;
}

/**
 * Structure for describing dropped packets
 */
struct dropped_packet_event {
    uint32_t id;              ///< Responder's ID
    uint32_t sequence;        ///< The stage the packet was dropped in
    struct uwb_counts events; ///< DW1000 device events
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
        const struct ranging_event *event;             ///< RANGING_EVENT
        uint32_t dropped_neighbor;                     ///< NEIGHBOR_DROP
        const char *node_version;                      ///< START_EVENT
        const struct dropped_packet_event *drop_event; ///< UWB_RANGING_DROP
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
