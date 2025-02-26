/**
 * @file comms.h
 *
 * @brief
 *
 * @date 2/17/25
 *
 * @author tom
 */

#ifndef BELUGA_COMMS_H
#define BELUGA_COMMS_H

#include <beluga_message.h>
#include <stdbool.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

enum comms_transport_evt {
    COMMS_TRANSPORT_EVT_RX_RDY,
    COMMS_TRANSPORT_EVT_TX_RDY,
};

typedef void (*comms_transport_handler_t)(enum comms_transport_evt evt,
                                          void *context);

#include <serial/comms_uart.h>

struct at_command_static_entry;

union at_command_entry {
    const struct at_command_static_entry *entry;
};

struct comms;

typedef int (*at_command_handler)(const struct comms *comms, size_t argc,
                                  char const *const *argv);

struct at_command_static_entry {
    const char *command;
    at_command_handler handler;
};

#define AT_CMD_DEFINE(_command)                                                \
    static int at_##_command(const struct comms *comms, size_t argc,           \
                             char const *const *argv)
#define AT_CMD_REGISTER(_command)                                              \
    static const struct at_command_static_entry UTIL_CAT(_comms_,              \
                                                         _command) = {         \
        .command = (const char *)STRINGIFY(_command),                          \
        .handler = (at_command_handler)UTIL_CAT(at_, _command),                \
    };                                                                         \
    static const TYPE_SECTION_ITERABLE(                                        \
        union at_command_entry, UTIL_CAT(at_cmd_, _command), shell_root_cmds,  \
        UTIL_CAT(at_cmd_, _command)) = {.entry = &UTIL_CAT(_comms_, _command)}

#define AT_CMD_COND_REGISTER(_flag, _command) IF_ENABLED(_flag, (AT_CMD_REGISTER(_command)))

BUILD_ASSERT(!IS_ENABLED(CONFIG_SHELL),
             "Shell cannot be enabled when using AT commands");

struct comms_transport;

struct comms_transport_api {
    int (*init)(const struct comms_transport *transport, const void *config,
                comms_transport_handler_t evt_handler, void *context);
    int (*uninit)(const struct comms_transport *transport);
    int (*enable)(const struct comms_transport *transport, bool blocking_tx);
    int (*write)(const struct comms_transport *transport, const void *data,
                 size_t length, size_t *cnt);
    int (*read)(const struct comms_transport *transport, void *data,
                size_t length, size_t *cnt);
    void (*update)(const struct comms_transport *transport);
};

struct comms_transport {
    const struct comms_transport_api *api;
    void *ctx;
};

enum comms_signal { COMMS_SIGNAL_RXRDY, COMMS_SIGNAL_TXDONE, COMMS_SIGNALS };

struct comms_buf {
    uint8_t buf[256];
    size_t len;
};

struct comms_ctx {
    struct comms_buf rx_buf;
    struct comms_buf tx_buf;

    struct k_poll_signal signals[COMMS_SIGNALS];
    struct k_poll_event events[COMMS_SIGNALS];
    struct k_mutex wr_mtx;
    k_tid_t tid;
};

struct comms {
    const struct comms_transport *iface;
    struct comms_ctx *ctx;
    const char *name;
    struct k_thread *thread;
    k_thread_stack_t *stack;
};

#define COMMS_DEFINE(_name, _transport)                                        \
    static const struct comms _name;                                           \
    static struct comms_ctx UTIL_CAT(_name, _ctx);                             \
    static K_KERNEL_STACK_DEFINE(_name##_stack, CONFIG_COMMANDS_STACK_SIZE);   \
    static struct k_thread _name##_thread;                                     \
    static const STRUCT_SECTION_ITERABLE(comms, _name) = {                     \
        .iface = (_transport),                                                 \
        .ctx = &UTIL_CAT(_name, _ctx),                                         \
        .name = STRINGIFY(_name),                                              \
        .thread = &_name##_thread,                                             \
        .stack = _name##_stack}

int comms_init(const struct comms *comms, const void *transport_config);
void comms_process(const struct comms *comms);

void at_msg(const struct comms *comms, const char *msg);
void __printflike(2, 3)
    at_msg_fmt(const struct comms *comms, const char *msg, ...);
int write_message_frame(const struct comms *comms,
                        const struct beluga_msg *msg);
void comms_flush_out(const struct comms *comms, int ret);

#define Z_OK_MSG_NOARGS(_comms, _msg)                                          \
    do {                                                                       \
        at_msg(_comms, _msg);                                                  \
        return 0;                                                              \
    } while (0)

#define Z_OK_MSG_ARGS(_comms, _msg, ...)                                       \
    do {                                                                       \
        at_msg_fmt(_comms, _msg, __VA_ARGS__);                                 \
        return 0;                                                              \
    } while (0)

#define Z_OK_MSG(_comms, _msg, ...)                                            \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (Z_OK_MSG_NOARGS(_comms, _msg)),        \
                (Z_OK_MSG_ARGS(_comms, _msg, __VA_ARGS__)))

#define OK(_comms, ...)                                                        \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (return 0),                             \
                (Z_OK_MSG(_comms, GET_ARG_N(1, __VA_ARGS__),                   \
                          GET_ARGS_LESS_N(1, __VA_ARGS__))))

#define Z_NOW_MSG_NOARGS(_comms, _msg)    at_msg(_comms, _msg)
#define Z_NOW_MSG_ARGS(_comms, _msg, ...) at_msg_fmt(_comms, _msg, __VA_ARGS__)
#define Z_NOW_MSG(_comms, _msg, ...)                                           \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (Z_NOW_MSG_NOARGS(_comms, _msg)),       \
                (Z_NOW_MSG_ARGS(_comms, _msg, __VA_ARGS__)))
#define Z_INSERT_MSG_NOW(_comms, ...)                                          \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (),                                     \
                (Z_NOW_MSG(_comms, GET_ARG_N(1, __VA_ARGS__),                  \
                           GET_ARGS_LESS_N(1, __VA_ARGS__));))

#define OK_NOW(_comms, ...)                                                    \
    do {                                                                       \
        Z_INSERT_MSG_NOW(_comms, __VA_ARGS__) comms_flush_out(_comms, 0);      \
    } while (0)

#define Z_ERR_MSG_NOARGS(_comms, _msg)                                         \
    do {                                                                       \
        at_msg(_comms, _msg);                                                  \
        return 1;                                                              \
    } while (0)
#define Z_ERR_MSG_ARGS(_comms, _msg, ...)                                      \
    do {                                                                       \
        at_msg_fmt(_comms, _msg, __VA_ARGS__);                                 \
        return 1;                                                              \
    } while (0)
#define ERROR(_comms, _msg, ...)                                               \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (Z_ERR_MSG_NOARGS(_comms, _msg)),       \
                (Z_ERR_MSG_ARGS(_comms, _msg, __VA_ARGS__)))

#define Z_ERR_MSG_NOARGS_NORET(_comms, _msg) at_msg(_comms, _msg)
#define Z_ERR_MSG_ARGS_NORET(_comms, _msg, ...)                                \
    at_msg_fmt(_comms, _msg, __VA_ARGS__)
#define ERROR_NORET(_comms, _msg, ...)                                         \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (Z_ERR_MSG_NOARGS_NORET(_comms, _msg)), \
                (Z_ERR_MSG_ARGS_NORET(_comms, _msg, __VA_ARGS__)))

#endif // BELUGA_COMMS_H
