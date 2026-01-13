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

template <typename NeighborType = BelugaSerial::BelugaNeighbor>
static void neighbor_list_update(struct beluga_serial *obj,
                                 const std::vector<NeighborType> &evt) {
    if (!obj) {
        return;
    }
    std::vector<struct beluga_neighbor> neighbors(evt.size());

    for (size_t i = 0; i < evt.size(); i++) {
        neighbors[i] = {
            .rssi = evt[i].rssi(),
            .id = evt[i].id(),
            .exchange = evt[i].exchange(),
            .range = evt[i].range(),
            .time = evt[i].time(),
        };
    }

    obj->_attr.neighbor_list_updates(neighbors.data(), neighbors.size());
}

template <typename NeighborType = BelugaSerial::BelugaNeighbor>
static void range_update(struct beluga_serial *obj,
                         const std::vector<NeighborType> &evt) {
    if (!obj) {
        return;
    }
    std::vector<struct beluga_neighbor> neighbors(evt.size());

    for (size_t i = 0; i < evt.size(); i++) {
        neighbors[i] = {
            .rssi = evt[i].rssi(),
            .id = evt[i].id(),
            .exchange = evt[i].exchange(),
            .range = evt[i].range(),
            .time = evt[i].time(),
        };
    }

    obj->_attr.neighbor_ranging_updates(neighbors.data(), neighbors.size());
}

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

static void resync_cb_(struct beluga_serial *obj) {
    if (!obj || !obj->_attr.resync) {
        return;
    }
    obj->_attr.resync();
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

    BelugaSerial::NeighborCallbacks<NEIGHBOR_CLASS> neighbor_cb_ = {
        .neighbor_update_cb =
            [obj](auto const &evt) { neighbor_list_update(obj, evt); },
        .range_updates_cb = [obj](auto const &evt) { range_update(obj, evt); },
    };

    obj->ctx = (void *)(new BelugaSerial::BelugaSerial<NEIGHBOR_CLASS>(
        attr_, neighbor_cb_));

    if (!obj->ctx) {
        free(obj);
        return nullptr;
    }

    if (attr->resync != nullptr) {
        static_cast<BelugaSerial::BelugaSerial<NEIGHBOR_CLASS> *>(obj->ctx)
            ->register_resync_cb([obj]() { resync_cb_(obj); });
    }

    return obj;
}

void destroy_beluga_serial_instance(struct beluga_serial **obj) {
    if (!obj || *obj == nullptr) {
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
    try {
        _CALL_API_FUNC(obj, start);
    } catch (const std::runtime_error &error) {
        printf("Start serial communication failed: %s\n", error.what());
        exit(EXIT_FAILURE);
    }
}

void beluga_serial_stop(struct beluga_serial *obj) {
    _CALL_API_FUNC(obj, stop);
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

void beluga_serial_starve(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, starve, arg);
}

void beluga_serial_exchange(struct beluga_serial *obj, const char *arg) {
    _CALL_AT_CMD(obj, exchange, arg);
}

void beluga_serial_reason(struct beluga_serial *obj) {
    _CALL_AT_CMD2(obj, reason);
}

static bool allocate_port_entry(
    const BelugaSerial::BelugaSerial<NEIGHBOR_CLASS>::target_pair &target,
    const std::string &port, struct beluga_serial_ports &beluga_port) {
    char *manf = (char *)calloc(target.first.length() + 1, sizeof(char));
    beluga_port.manufacturer = manf;

    if (beluga_port.manufacturer == nullptr) {
        return false;
    }

    strncpy(manf, target.first.c_str(), target.first.length());

    char *product = (char *)calloc(target.second.length() + 1, sizeof(char));
    beluga_port.product = product;

    if (beluga_port.product == nullptr) {
        free(manf);
        return false;
    }

    strncpy(product, target.second.c_str(), target.second.length());

    char *port_ = (char *)calloc(port.length() + 1, sizeof(char));
    beluga_port.port = port_;

    if (beluga_port.port == nullptr) {
        free(manf);
        free(product);
        return false;
    }

    strncpy(port_, port.c_str(), port.length());

    return true;
}

struct beluga_serial_ports *find_ports(size_t *len) {
    if (len == nullptr) {
        return nullptr;
    }

    std::map<BelugaSerial::BelugaSerial<NEIGHBOR_CLASS>::target_pair,
             std::vector<std::string>>
        ports_;
    BelugaSerial::find_ports(ports_);

    size_t num_ports = 0;
    for (const auto &[fst, snd] : ports_) {
        num_ports += snd.size();
    }

    if (num_ports == 0) {
        *len = 0;
        return nullptr;
    }

    struct beluga_serial_ports *ports = (struct beluga_serial_ports *)calloc(
        num_ports, sizeof(struct beluga_serial_ports));

    if (ports == nullptr) {
        *len = 0;
        return nullptr;
    }

    size_t current = 0;
    for (const auto &[fst, snd] : ports_) {
        for (const auto &port : snd) {
            if (!allocate_port_entry(fst, port, ports[current])) {
                cleanup_find_ports(&ports, num_ports);
                *len = 0;
                return nullptr;
            }
            current++;
        }
    }

    *len = num_ports;
    return ports;
}

void cleanup_find_ports(struct beluga_serial_ports **ports, size_t len) {
    if (ports == nullptr || *ports == nullptr) {
        return;
    }

    struct beluga_serial_ports *ports_ = *ports;
    for (size_t i = 0; i < len; i++) {
        if (ports_[i].manufacturer != nullptr) {
            free((char *)ports_[i].manufacturer);
        }

        if (ports_[i].product != nullptr) {
            free((char *)ports_[i].product);
        }

        if (ports_[i].port != nullptr) {
            free((char *)ports_[i].port);
        }
    }

    free(ports_);
    *ports = nullptr;
}
};
