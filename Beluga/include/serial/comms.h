/**
 * @file comms.h
 *
 * @brief
 *
 * @date 2/17/25
 *
 * @author tom
 */

#ifndef BELUGA_DTS_COMMS_H
#define BELUGA_DTS_COMMS_H

#include <stdbool.h>
#include <stddef.h>

enum comms_transport_evt {
    COMMS_TRANSPORT_EVT_RX_RDY,
    COMMS_TRANSPORT_EVT_TX_RDY,
};

typedef void (*comms_transport_handler_t)(enum comms_transport_evt evt,
                                          void *context);

struct comms_transport;

struct comms_transport_api {
    int (*init)(const struct comms_transport *transport, const void *config, comms_transport_handler_t evt_handler, void *context);
    int (*uninit)(const struct comms_transport *transport);
    int (*enable)(const struct comms_transport *transport, bool blocking_tx);
    int (*write)(const struct comms_transport *transport, const void *data, size_t length, size_t *cnt);
    int (*read)(const struct comms_transport *transport, void *data, size_t length, size_t *cnt);
    void (*update)(const struct comms_transport *transport);
};

struct comms_transport {
    const struct comms_transport_api *api;
    void *ctx;
};

#endif // BELUGA_DTS_COMMS_H
