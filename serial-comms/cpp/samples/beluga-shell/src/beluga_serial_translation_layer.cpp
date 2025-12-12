/**
 * @file beluga_serial_translation_layer.cpp
 *
 * @brief
 *
 * @date 12/12/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <beluga/beluga_serial.hpp>
#include <beluga_serial_c_api.h>

#define NEIGHBOR_CLASS

extern "C" {
#include <stdlib.h>

static void range_event_cb_(struct beluga_serial *obj,
                            const BelugaSerial::RangeEvent &evt) {
    if (!obj || !obj->_attr.range_events) {
        return;
    }
    struct range_event event = {evt.ID, evt.EXCHANGE, evt.TIMESTAMP};
    obj->_attr.range_events(&event);
}

static int logger_cb_(struct beluga_serial *obj, const char *fmt,
                      va_list args) {
    if (!obj || !obj->_attr.logger) {
        return 0;
    }
    return obj->_attr.logger(fmt, args);
}

static void
report_uwb_drop_cb_(struct beluga_serial *obj,
                    const BelugaSerial::BelugaFrame::DroppedUwbExchange &evt) {
    if (!obj || !obj->_attr.uwb_drop_events) {
        return;
    }
    struct dropped_uwb_exchange event = {
        evt.ID,
        evt.STAGE,
    };
    obj->_attr.uwb_drop_events(&event);
}

static void reboot_event_cb_(struct beluga_serial *obj) {
    if (!obj || !obj->_attr.reboot_event) {
        return;
    }
    obj->_attr.reboot_event();
}

static void fatal_error_cb_(struct beluga_serial *obj, const std::string &msg) {
    if (!obj || !obj->_attr.fatal_error_event) {
        return;
    }
    obj->_attr.fatal_error_event(msg.c_str());
}

struct beluga_serial *
create_beluga_serial_instance(const struct beluga_serial_attr *attr) {
    if (attr == nullptr) {
        return nullptr;
    }
    auto *obj = (struct beluga_serial *)malloc(sizeof(struct beluga_serial));

    if (obj == nullptr) {
        return nullptr;
    }

    obj->_attr = *attr;

    BelugaSerial::BelugaSerialAttributes attr_ = {
        .port = obj->_attr.port,
        .baud = obj->_attr.baud,
        .timeout = std::chrono::milliseconds(obj->_attr.timeout),
        .serial_timeout = std::chrono::milliseconds(obj->_attr.serial_timeout),
        .range_event_cb =
            [obj](const BelugaSerial::RangeEvent &evt) {
                range_event_cb_(obj, evt);
            },
        .logger_cb = [obj](const char *fmt,
                           va_list args) { return logger_cb_(obj, fmt, args); },
        .report_uwb_drops_cb =
            [obj](const BelugaSerial::BelugaFrame::DroppedUwbExchange &evt) {
                return report_uwb_drop_cb_(obj, evt);
            },
        .unexpected_reboot_event = [obj]() { reboot_event_cb_(obj); },
        .fatal_error_event =
            [obj](const std::string &msg) { fatal_error_cb_(obj, msg); },
    };

    BelugaSerial::NeighborCallbacks<NEIGHBOR_CLASS> neighbor_cb_ = {};

    obj->ctx = (void *)(new BelugaSerial::BelugaSerial<NEIGHBOR_CLASS>(
        attr_, neighbor_cb_));

    if (!obj->ctx) {
        free(obj);
        return nullptr;
    }

    return obj;
}

void destroy_beluga_serial_instance(struct beluga_serial **obj) {
    if (!obj) {
        return;
    }

    delete (
        static_cast<BelugaSerial::BelugaSerial<NEIGHBOR_CLASS> *>((*obj)->ctx));
    free(*obj);
    *obj = nullptr;
}

#define _CALL_API_FUNC(obj_, func_)                                            \
    do {                                                                       \
        if (!obj_) {                                                           \
            return;                                                            \
        }                                                                      \
        auto serial =                                                          \
            static_cast<BelugaSerial::BelugaSerial<NEIGHBOR_CLASS> *>(         \
                obj_->ctx);                                                    \
        serial->func_();                                                       \
    } while (false)

void beluga_serial_start(struct beluga_serial *obj) {
    _CALL_API_FUNC(obj, start);
}

void beluga_serial_stop(struct beluga_serial *obj) {
    try {
        _CALL_API_FUNC(obj, stop);
    } catch (std::future_error &err) {
    }
}

void beluga_serial_close(struct beluga_serial *obj) {
    _CALL_API_FUNC(obj, close);
}

#define _CALL_AT_CMD(obj_, func_, arg_)                                        \
    do {                                                                       \
        if (!obj) {                                                            \
            return;                                                            \
        }                                                                      \
        memset(obj_->response, 0, sizeof(obj_->response));                     \
        auto serial =                                                          \
            static_cast<BelugaSerial::BelugaSerial<NEIGHBOR_CLASS> *>(         \
                obj_->ctx);                                                    \
        std::string response;                                                  \
        if (!arg_) {                                                           \
            response = serial->func_();                                        \
        } else {                                                               \
            response = serial->func_(arg_);                                    \
        }                                                                      \
        memcpy(obj_->response, response.c_str(), response.length());           \
    } while (false)

#define _CALL_AT_CMD2(obj_, func_)                                             \
    do {                                                                       \
        if (!obj) {                                                            \
            return;                                                            \
        }                                                                      \
        memset(obj_->response, 0, sizeof(obj_->response));                     \
        auto serial =                                                          \
            static_cast<BelugaSerial::BelugaSerial<NEIGHBOR_CLASS> *>(         \
                obj_->ctx);                                                    \
        std::string response;                                                  \
        response = serial->func_();                                            \
        memcpy(obj_->response, response.c_str(), response.length());           \
    } while (false)

void start_ble(struct beluga_serial *obj) { _CALL_AT_CMD2(obj, start_ble); }

void stop_ble(struct beluga_serial *obj) { _CALL_AT_CMD2(obj, stop_ble); }
};
