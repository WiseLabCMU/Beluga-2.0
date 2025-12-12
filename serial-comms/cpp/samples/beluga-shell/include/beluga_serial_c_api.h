/**
 * @file beluga_serial_c_api.h
 *
 * @brief
 *
 * @date 12/12/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_BELUGA_SERIAL_C_API_H
#define BELUGA_BELUGA_SERIAL_C_API_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAX_RESPONSE_SIZE
#define MAX_RESPONSE_SIZE 512
#endif

#include <serial/core/C-API/serial_common.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

struct range_event {
    uint16_t id;
    uint32_t exchange;
    int64_t timestamp;
};

struct dropped_uwb_exchange {
    uint16_t id;
    uint32_t stage;
};

struct beluga_neighbor {
    int8_t rssi;
    uint16_t id;
    uint32_t exchange;
    double range;
    int64_t time;
};

enum uwb_amp_stage {
    BOOST_NORM = 0,
    BOOSTP_500 = 1,
    BOOSTP_250 = 2,
    BOOSTP_125 = 3,
};

typedef void (*range_event_cb)(const struct range_event *event);
typedef int (*logger_cb)(const char *msg, va_list args);
typedef void (*uwb_drop_cb)(const struct dropped_uwb_exchange *event);
typedef void (*unexpected_reboot_cb)(void);
typedef void (*fatal_error_cb)(const char *msg);
typedef void (*neighbor_update_cb)(const struct beluga_neighbor *updates,
                                   size_t len);
typedef void (*range_update_cb)(const struct beluga_neighbor *updates,
                                size_t len);
typedef void (*resync_cb)(void);

struct beluga_serial_attr {
    const char *port;
    enum BaudRate baud;
    uint64_t timeout;
    uint64_t serial_timeout;
    range_event_cb range_events;
    logger_cb logger;
    uwb_drop_cb uwb_drop_events;
    unexpected_reboot_cb reboot_event;
    fatal_error_cb fatal_error_event;
    neighbor_update_cb neighbor_list_updates;
    range_update_cb neighbor_ranging_updates;
};

struct beluga_serial {
    void *ctx;
    char response[MAX_RESPONSE_SIZE];
    struct beluga_serial_attr _attr;
};

struct beluga_serial *
create_beluga_serial_instance(const struct beluga_serial_attr *attr);
void destroy_beluga_serial_instance(struct beluga_serial **obj);
void beluga_serial_start(struct beluga_serial *obj);
void beluga_serial_stop(struct beluga_serial *obj);
void beluga_serial_close(struct beluga_serial *obj);
// void swap_port(struct beluga_serial *obj, const char *port);
void start_ble(struct beluga_serial *obj);
void stop_ble(struct beluga_serial *obj);
// void start_uwb(struct beluga_serial *obj);
// void stop_uwb(struct beluga_serial *obj);
// void id(struct beluga_serial *obj, const char *arg);
// void bootmode(struct beluga_serial *obj, const char *arg);
// void rate(struct beluga_serial *obj, const char *arg);
// void channel(struct beluga_serial *obj, const char *arg);
// void reset(struct beluga_serial *obj);
// void timeout(struct beluga_serial *obj, const char *arg);
// void txpower(struct beluga_serial *obj, const char *arg);
// void txpower2(struct beluga_serial *obj, enum uwb_amp_stage stage, uint32_t
// coarse, uint32_t fine); void streammode(struct beluga_serial *obj, const char
// *arg); void twrmode(struct beluga_serial *obj, const char *arg); void
// ledmode(struct beluga_serial *obj, const char *arg); void reboot(struct
// beluga_serial *obj); void pwramp(struct beluga_serial *obj, const char *arg);
// void antenna(struct beluga_serial *obj, const char *arg);
// void time(struct beluga_serial *obj);
// void deepsleep(struct beluga_serial *obj);
// void datarate(struct beluga_serial *obj, const char *arg);
// void preamble(struct beluga_serial *obj, const char *arg);
// void pulserate(struct beluga_serial *obj, const char *arg);
// void phr(struct beluga_serial *obj, const char *arg);
// void pac(struct beluga_serial *obj, const char *arg);
// void sfd(struct beluga_serial *obj, const char *arg);
// void panid(struct beluga_serial *obj, const char *arg);
// void evict(struct beluga_serial *obj, const char *arg);
// void verbose(struct beluga_serial *obj, const char *arg);
// void status(struct beluga_serial *obj);
// void version(struct beluga_serial *obj);
// void exchange(struct beluga_serial *obj, const char *arg);

#ifdef __cplusplus
};
#endif

#endif // BELUGA_BELUGA_SERIAL_C_API_H