/**
 * @file comms.c
 *
 * @brief
 *
 * @date 2/18/25
 *
 * @author tom
 */

#include <ctype.h>
#include <serial/comms.h>
#include <utils.h>
#include <zephyr/kernel.h>

#define _COMMS_API(_comms, _func, ...)                                         \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__),                                         \
                (comms->iface->api->_func(_comms->iface)),                     \
                (comms->iface->api->_func(_comms->iface, __VA_ARGS__)))

#define _COMMS_BUF_APPEND(_comms, data)                                        \
    do {                                                                       \
        (_comms)->ctx->rx_buf.buf[(_comms)->ctx->rx_buf.len] = data;           \
        BOUND_INCREMENT((_comms)->ctx->rx_buf.len,                             \
                        ARRAY_SIZE((_comms)->ctx->rx_buf.buf));                \
        (_comms)->ctx->rx_buf.buf[(_comms)->ctx->rx_buf.len] = '\0';           \
    } while (0)

// TODO: Make into config
#define MAX_TOKENS 20

typedef void (*comms_signal_handler_t)(const struct comms *comms);

static void comms_signal_handle(const struct comms *comms,
                                enum comms_signal sig_idx,
                                comms_signal_handler_t handler) {
    struct k_poll_signal *sig = &comms->ctx->signals[sig_idx];
    int set;
    int res;

    k_poll_signal_check(sig, &set, &res);

    if (set) {
        k_poll_signal_reset(sig);
        handler(comms);
    }
}

static void cmd_buffer_clear(const struct comms *comms) {
    comms->ctx->rx_buf.buf[0] = '\0';
    comms->ctx->rx_buf.len = 0;
}

/**
 * @brief Tokenizes the input string
 *
 * @param[in] s The string to split into tokens
 * @param[out] argv The tokens found from the string
 *
 * @return The number of tokens found
 *
 * @note This function does not handle quoted arguments
 */
static size_t argparse(char *s, char **argv) {
    char *temp;
    size_t argc;

    for (argc = 0, temp = s; argc < (MAX_TOKENS - 1); argc++) {
        while (isspace((int)*temp)) {
            temp++;
        }

        if (*temp == '\0') {
            break;
        }

        argv[argc] = temp;

        while (isgraph((int)*temp)) {
            temp++;
        }

        if (isspace((int)*temp)) {
            *temp = '\0';
            temp++;
        }
    }

    return argc;
}

static inline const union at_command_entry *root_cmd_get(uint32_t id) {
    const union at_command_entry *cmd;

    TYPE_SECTION_GET(union at_command_entry, shell_root_cmds, id, &cmd);

    return cmd;
}

static inline size_t root_cmd_count(void) {
    size_t len;

    TYPE_SECTION_COUNT(union at_command_entry, shell_root_cmds, &len);

    return len;
}

static inline const struct at_command_static_entry *at_cmd_get(size_t idx) {
    return (idx < root_cmd_count()) ? root_cmd_get(idx)->entry : NULL;
}

static const struct at_command_static_entry *find_at_cmd(const char *cmd_str) {
    const struct at_command_static_entry *entry;
    size_t idx = 0;

    while ((entry = at_cmd_get(idx++)) != NULL) {
        if (strcmp(cmd_str, entry->command) == 0) {
            return entry;
        }
    }

    return NULL;
}

static int execute_command(const struct comms *comms, size_t argc,
                           const char **argv,
                           const struct at_command_static_entry *entry) {

    if (entry->handler == NULL) {
        // TODO: Error not implemented
        return -ENOTSUP;
    }
    return entry->handler(comms, argc, argv);
}

static int execute(const struct comms *comms) {
    char *argv[MAX_TOKENS + 1] = {0};
    size_t argc = 0;
    const struct at_command_static_entry *entry;

    argc = argparse(comms->ctx->rx_buf.buf, argv);
    argv[argc] = NULL;

    if (0 != strncmp((const char *)comms->ctx->rx_buf.buf, "AT+", 3)) {
        if (0 == strncmp((const char *)comms->ctx->rx_buf.buf, "AT", 2)) {
            // TODO: Error: Only input AT without + command
        } else {
            // TODO: Error: Not an AT command
        }
        return -ENOEXEC;
    }

    if (comms->ctx->rx_buf.len == 3) {
        // TODO: Error: No command found after AT+
        return -ENOEXEC;
    }

    entry = find_at_cmd(argv[0] + 3);

    if (entry == NULL) {
        // TODO: Error: ERROR Invalid AT Command
        return -ENOEXEC;
    }

    return execute_command(comms, argc, (const char **)argv, entry);
}

void comms_thread(void *comms_handle, void *p2, void *p3) {
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    struct comms *comms = comms_handle;
    int err;

    err = _COMMS_API(comms, enable, false);
    if (err != 0) {
        return;
    }

    while (true) {
        err = k_poll(comms->ctx->events, COMMS_SIGNAL_TXDONE, K_FOREVER);

        if (err != 0) {
            k_mutex_lock(&comms->ctx->wr_mtx, K_FOREVER);
            // TODO: Report err
            k_mutex_unlock(&comms->ctx->wr_mtx);
            return;
        }

        k_mutex_lock(&comms->ctx->wr_mtx, K_FOREVER);
        comms_signal_handle(comms, COMMS_SIGNAL_RXRDY, comms_process);
        _COMMS_API(comms, update);
        k_mutex_unlock(&comms->ctx->wr_mtx);
    }
    __ASSERT_UNREACHABLE;
}

static void transport_evt_handler(enum comms_transport_evt evt_type,
                                  void *ctx) {
    struct comms *comms = ctx;
    struct k_poll_signal *signal;

    signal = (evt_type == COMMS_TRANSPORT_EVT_RX_RDY)
                 ? &comms->ctx->signals[COMMS_SIGNAL_RXRDY]
                 : &comms->ctx->signals[COMMS_SIGNAL_TXDONE];
    k_poll_signal_raise(signal, 0);
}

static int instance_init(const struct comms *comms,
                         const void *transport_config) {
    memset(comms->ctx, 0, sizeof(*comms->ctx));

    k_mutex_init(&comms->ctx->wr_mtx);

    for (size_t i = 0; i < COMMS_SIGNALS; i++) {
        k_poll_signal_init(&comms->ctx->signals[i]);
        k_poll_event_init(&comms->ctx->events[i], K_POLL_TYPE_SIGNAL,
                          K_POLL_MODE_NOTIFY_ONLY, &comms->ctx->signals[i]);
    }

    return _COMMS_API(comms, init, transport_config, transport_evt_handler,
                      (void *)comms);
}

int comms_init(const struct comms *comms, const void *transport_config) {
    __ASSERT_NO_MSG(comms);
    __ASSERT_NO_MSG(comms->ctx && comms->iface);

    if (comms->ctx->tid) {
        return -EALREADY;
    }

    int err = instance_init(comms, transport_config);

    if (err != 0) {
        return err;
    }

    k_tid_t tid = k_thread_create(
        comms->thread, comms->stack, CONFIG_COMMANDS_STACK_SIZE, comms_thread,
        (void *)comms, NULL, NULL, CONFIG_BELUGA_COMMANDS_PRIO, 0, K_NO_WAIT);
    comms->ctx->tid = tid;
    k_thread_name_set(tid, comms->name);

    return 0;
}

void comms_process(const struct comms *comms) {
    __ASSERT_NO_MSG(comms);
    __ASSERT_NO_MSG(comms->ctx);

    size_t count = 0;
    char data;
    struct comms_buf *buf = &comms->ctx->rx_buf;

    while (true) {
        (void)_COMMS_API(comms, read, &data, sizeof(data), &count);
        if (count == 0) {
            // No data
            return;
        }

        if ((data == '\n' || data == '\r') && buf->len != 0) {
            int ret = execute(comms);
            cmd_buffer_clear(comms);
            continue;
        }
        if (isprint((int)data) != 0) {
            _COMMS_BUF_APPEND(comms, data);
        }
    }
}
