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

int beluga_serial_swap_port(struct beluga_serial *obj, const char *port) {
    if (!obj) {
        return -EINVAL;
    }

    auto serial =
        static_cast<BelugaSerial::BelugaSerial<NEIGHBOR_CLASS> *>(obj->ctx);

    try {
        serial->swap_port(port);
    } catch (const std::invalid_argument &) {
        return -EINVAL;
    } catch (const std::runtime_error &) {
        return -EBUSY;
    }

    return 0;
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
        memcpy(obj_->response, response.c_str(),                               \
               std::min(response.length(), sizeof(obj->response) - 1));        \
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
        memcpy(obj_->response, response.c_str(),                               \
               std::min(response.length(), sizeof(obj->response) - 1));        \
    } while (false)

void beluga_serial_start_ble(struct beluga_serial *obj) {
    _CALL_AT_CMD2(obj, start_ble);
}

void beluga_serial_stop_ble(struct beluga_serial *obj) {
    _CALL_AT_CMD2(obj, stop_ble);
}

void beluga_serial_start_uwb(struct beluga_serial *obj) {
    _CALL_AT_CMD2(obj, start_uwb);
}

void beluga_serial_stop_uwb(struct beluga_serial *obj) {
    _CALL_AT_CMD2(obj, stop_uwb);
}

void beluga_serial_id(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, id, arg);
}

void beluga_serial_bootmode(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, bootmode, arg);
}

void beluga_serial_rate(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, rate, arg);
}

void beluga_serial_channel(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, channel, arg);
}

void beluga_serial_reset(struct beluga_serial *obj) {
    _CALL_AT_CMD2(obj, reset);
}

void beluga_serial_timeout(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, timeout, arg);
}

void beluga_serial_txpower(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, txpower, arg);
}

void beluga_serial_txpower2(struct beluga_serial *obj, enum uwb_amp_stage stage,
                            uint32_t coarse, uint32_t fine) {
    if (!obj) {
        return;
    }

    auto serial =
        static_cast<BelugaSerial::BelugaSerial<NEIGHBOR_CLASS> *>(obj->ctx);
    memset(obj->response, 0, sizeof(obj->response));
    BelugaSerial::BelugaSerial<NEIGHBOR_CLASS>::UwbAmplificationStage stage_;

    switch (stage) {
    case BOOST_NORM:
        stage_ = BelugaSerial::BelugaSerial<NEIGHBOR_CLASS>::BOOST_NORM;
        break;
    case BOOSTP_500:
        stage_ = BelugaSerial::BelugaSerial<NEIGHBOR_CLASS>::BOOSTP_500;
        break;
    case BOOSTP_250:
        stage_ = BelugaSerial::BelugaSerial<NEIGHBOR_CLASS>::BOOSTP_250;
        break;
    case BOOSTP_125:
        stage_ = BelugaSerial::BelugaSerial<NEIGHBOR_CLASS>::BOOSTP_125;
        break;
    default:
        return;
    }

    std::string response = serial->txpower(stage_, coarse, fine);
    memcpy(obj->response, response.c_str(),
           std::min(response.length(), sizeof(obj->response) - 1));
}

void beluga_serial_streammode(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, streammode, arg);
}

void beluga_serial_twrmode(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, twrmode, arg);
}

void beluga_serial_ledmode(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, ledmode, arg);
}

void beluga_serial_reboot(struct beluga_serial *obj) {
    _CALL_AT_CMD2(obj, reboot);
}

void beluga_serial_pwramp(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, pwramp, arg);
}

void beluga_serial_antenna(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, antenna, arg);
}

void beluga_serial_time(struct beluga_serial *obj) { _CALL_AT_CMD2(obj, time); }

void beluga_serial_deepsleep(struct beluga_serial *obj) {
    _CALL_AT_CMD2(obj, deepsleep);
}

void beluga_serial_datarate(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, datarate, arg);
}

void beluga_serial_preamble(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, preamble, arg);
}

void beluga_serial_pulserate(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, pulserate, arg);
}

void beluga_serial_phr(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, phr, arg);
}

void beluga_serial_pac(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, pac, arg);
}

void beluga_serial_sfd(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, sfd, arg);
}

void beluga_serial_panid(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, panid, arg);
}

void beluga_serial_evict(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, evict, arg);
}

void beluga_serial_verbose(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, verbose, arg);
}

void beluga_serial_status(struct beluga_serial *obj) {
    _CALL_AT_CMD2(obj, status);
}

void beluga_serial_version(struct beluga_serial *obj) {
    _CALL_AT_CMD2(obj, version);
}

void beluga_serial_exchange(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, exchange, arg);
}
};
