/**
 * @file comms.c
 *
 * @brief
 *
 * @date 2/18/25
 *
 * @author tom
 */

#include <beluga_message.h>
#include <ctype.h>
#include <serial/comms.h>
#include <stdio.h>
#include <unistd.h>
#include <utils.h>
#include <watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/cbprintf.h>

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

#define MAX_TOKENS CONFIG_COMMS_MAX_TOKENS

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
    comms->ctx->tx_buf.buf[0] = '\0';
    comms->ctx->tx_buf.len = 0;
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
        at_msg(comms, "Not implemented");
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
            at_msg(comms, "Only input AT without + command");
        } else {
            at_msg(comms, "Not an AT command");
        }
        return -ENOEXEC;
    }

    if (comms->ctx->rx_buf.len == 3) {
        at_msg(comms, "No command found after AT+");
        return -ENOEXEC;
    }

    entry = find_at_cmd(argv[0] + 3);

    if (entry == NULL) {
        at_msg(comms, "Invalid AT command");
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
        // k_sleep(K_MSEC(100));
        err = k_poll(comms->ctx->events, COMMS_SIGNAL_TXDONE, K_FOREVER);

        if (err != 0) {
            k_mutex_lock(&comms->ctx->wr_mtx, K_FOREVER);
            // TODO: Report err
            k_mutex_unlock(&comms->ctx->wr_mtx);
            return;
        }

        comms_signal_handle(comms, COMMS_SIGNAL_RXRDY, comms_process);
        if (comms->iface->api->update) {
            k_mutex_lock(&comms->ctx->wr_mtx, K_FOREVER);
            _COMMS_API(comms, update);
            k_mutex_unlock(&comms->ctx->wr_mtx);
        }
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

static void at_respond(const struct comms *comms, bool ok);

void comms_process(const struct comms *comms) {
    __ASSERT_NO_MSG(comms);
    __ASSERT_NO_MSG(comms->ctx);

    size_t count = 0;
    char data;
    struct comms_buf *buf = &comms->ctx->rx_buf;

    struct task_wdt_attr watchdog = {.period = 2000};
    if (spawn_task_watchdog(&watchdog) < 0) {
        printk("Unable to spawn task watchdog in command thread\n");
        return;
    }

    while (true) {
        watchdog_red_rocket(&watchdog);
        (void)_COMMS_API(comms, read, &data, sizeof(data), &count);
        if (count == 0) {
            // No data
            break;
        }

        if ((data == '\n' || data == '\r') && buf->len != 0) {
            int ret = execute(comms);
            at_respond(comms, ret == 0);
            cmd_buffer_clear(comms);
            continue;
        }
        if (isprint((int)data) != 0) {
            _COMMS_BUF_APPEND(comms, data);
        }
    }
    if (kill_task_watchdog(&watchdog) < 0) {
        printk("Unable to spawn task watchdog in command thread\n");
    }
}

static void comms_write(const struct comms *comms, const void *data,
                        size_t length) {
    __ASSERT_NO_MSG(comms && data);

    size_t offset = 0;
    size_t tmp_cnt;

    k_mutex_lock(&comms->ctx->wr_mtx, K_FOREVER);
    while (length) {
        int err = _COMMS_API(comms, write, &((const uint8_t *)data)[offset],
                             length, &tmp_cnt);
        (void)err;

        __ASSERT_NO_MSG(err == 0);
        __ASSERT_NO_MSG(length >= tmp_cnt);
        offset += tmp_cnt;
        length -= tmp_cnt;
    }
    k_mutex_unlock(&comms->ctx->wr_mtx);
}

static void at_respond(const struct comms *comms, bool ok) {
    __ASSERT(comms->ctx->tx_buf.len <= (sizeof(comms->ctx->tx_buf.buf) - 5),
             "Not enough room in TX buffer");

    if (ok) {
        if (comms->ctx->tx_buf.len != 0) {
            comms->ctx->tx_buf.buf[comms->ctx->tx_buf.len++] = ' ';
        }
        comms->ctx->tx_buf.buf[comms->ctx->tx_buf.len++] = 'O';
        comms->ctx->tx_buf.buf[comms->ctx->tx_buf.len++] = 'K';
    }

    if (comms->ctx->format == FORMAT_FRAMES) {
        comms->ctx->tx_buf.buf[comms->ctx->tx_buf.len] = '\0';
    } else {
        comms->ctx->tx_buf.buf[comms->ctx->tx_buf.len++] = '\r';
        comms->ctx->tx_buf.buf[comms->ctx->tx_buf.len++] = '\n';
    }

    struct beluga_msg msg = {.type = COMMAND_RESPONSE,
                             .payload.response =
                                 (const char *)comms->ctx->tx_buf.buf};

    comms_write_msg(comms, &msg);
}

void at_msg(const struct comms *comms, const char *msg) {
    char *buf = comms->ctx->tx_buf.buf;
    size_t msg_len = comms->ctx->tx_buf.len;
    const size_t buf_size =
        sizeof(comms->ctx->tx_buf.buf) - 5; // Leave room for " OK\r\n"
    const char *msg_ = msg;

    for (; msg_len < buf_size; msg_len++, msg_++) {
        if (*msg_ == '\r' || *msg_ == '\n') {
            // Ignore line endings
            msg_len--;
            continue;
        }
        if (*msg_ == '\0') {
            break;
        }
        buf[msg_len] = *msg_;
    }

    comms->ctx->tx_buf.len = msg_len;
}

static int out_func(int c, void *ctx) {
    const struct comms *comms = ctx;
    char c_str[2] = {(char)c, '\0'};
    at_msg(comms, c_str);
    return 0;
}

void at_msg_fmt(const struct comms *comms, const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    (void)cbvprintf(out_func, (void *)comms, msg, args);
    va_end(args);
}

void comms_flush_out(const struct comms *comms, int ret) {
    int set;
    int res;
    struct k_poll_signal *sig = &comms->ctx->signals[COMMS_SIGNAL_TXDONE];
    k_poll_signal_reset(sig);
    at_respond(comms, ret == 0);

    k_poll(&comms->ctx->events[COMMS_SIGNAL_TXDONE], 1, K_MSEC(500));
    k_poll_signal_check(sig, &set, &res);
}

int set_format(const struct comms *comms, enum comms_out_format_mode mode) {
    if (mode >= FORMAT_INVALID || comms == NULL) {
        return -EINVAL;
    }
    comms->ctx->format = mode;
    return 0;
}

#define HEADER_GEN(_comms, _msg)                                               \
    do {                                                                       \
        const char *header = _msg;                                             \
        size_t header_len = sizeof((uint8_t[]){_msg}) - 1;                     \
        comms_write(_comms, header, header_len);                               \
    } while (0)

#if defined(CONFIG_UWB_LOGIC_CLK)
#define HEADER        "ID,RSSI,RANGE,TIMESTAMP,EXCHANGE\r\n"
#define ASCII_FMT_STR "%" PRIu16 ",%" PRId8 ",%f,%" PRId64 ",%" PRIu32 "\r\n"
#define JSON_FMT_STR                                                           \
    "{ID:%" PRIu16 ",RSSI:%" PRId8 ",RANGE:%f,TIMESTAMP:%" PRId64              \
    ",EXCHANGE:%" PRIu32 "}\r\n"
#define FMT_PARAMS(_i)                                                         \
    list[(_i)].UUID, list[(_i)].RSSI, (double)list[(_i)].range,                \
        list[(_i)].time_stamp, list[(_i)].exchange_id
#else
#define HEADER        "ID,RSSI,RANGE,TIMESTAMP\r\n"
#define ASCII_FMT_STR "%" PRIu16 ",%" PRId8 ",%f,%" PRId64 "\r\n"
#define JSON_FMT_STR                                                           \
    "{ID:%" PRIu16 ",RSSI:%" PRId8 ",RANGE:%f,TIMESTAMP:%" PRId64 "}\r\n"
#define FMT_PARAMS(_i)                                                         \
    list[(_i)].UUID, list[(_i)].RSSI, (double)list[(_i)].range,                \
        list[(_i)].time_stamp
#endif

static int s_write_neighbors(const struct comms *comms,
                             const struct beluga_msg *msg) {
    __ASSERT_NO_MSG(comms && msg);
    bool stream = msg->payload.stream;
    bool header_printed = comms->ctx->format != FORMAT_ASCII;
    const struct node *list = msg->payload.neighbor_list;

    if (!list) {
        return -EINVAL;
    }

    for (size_t i = 0; i < MAX_ANCHOR_COUNT; i++) {
        if (list[i].UUID != 0 && (!stream || list[i].update_flag)) {
            char s[256];
            size_t len;
            if (comms->ctx->format == FORMAT_ASCII) {
                if (!header_printed) {
                    HEADER_GEN(comms, HEADER);
                    header_printed = true;
                }
                len = snprintf(s, sizeof(s) - 1, ASCII_FMT_STR, FMT_PARAMS(i));
            } else {
                len = snprintf(s, sizeof(s) - 1, JSON_FMT_STR, FMT_PARAMS(i));
            }
            comms_write(comms, s, len);
        }
    }

    return 0;
}

static int ascii_write_start_event(const struct comms *comms,
                                   const struct beluga_msg *msg) {
    const char *ending = "\r\n";
    const char *payload = msg->payload.node_version;

    if (!payload) {
        return -EINVAL;
    }

    for (; *payload != '\0'; payload++) {
        comms_write(comms, payload, 1);
    }

    comms_write(comms, ending, 2);
    return 0;
}

static int json_write_dropped_neighbor(const struct comms *comms,
                                       const struct beluga_msg *msg) {
    if (comms->ctx->format != FORMAT_JSON) {
        return -ENOTSUP;
    }

    char s[32];
    size_t len = snprintf(s, sizeof(s) - 1, "rm: %" PRId32 "\r\n",
                          msg->payload.dropped_neighbor);

    comms_write(comms, s, len);
    return 0;
}

static int comms_write_normal(const struct comms *comms,
                              const struct beluga_msg *msg) {
    __ASSERT_NO_MSG(comms && msg);
    int ret = 0;

    switch (msg->type) {
    case COMMAND_RESPONSE: {
        comms_write(comms, comms->ctx->tx_buf.buf, comms->ctx->tx_buf.len);
        break;
    }
    case NEIGHBOR_UPDATES: {
        ret = s_write_neighbors(comms, msg);
        break;
    }
    case START_EVENT: {
        ret = ascii_write_start_event(comms, msg);
        break;
    }
    case NEIGHBOR_DROP: {
        ret = json_write_dropped_neighbor(comms, msg);
        break;
    }
    default: {
        ret = -ECANCELED;
        break;
    }
    }

    return ret;
}

static int comms_write_frame(const struct comms *comms,
                             const struct beluga_msg *msg) {
    __ASSERT_NO_MSG(comms && msg);

    ssize_t len = frame_length(msg);
    if (len < 1) {
        return -EINVAL;
    }

    uint8_t *buffer = k_malloc(len + 1);
    if (buffer == NULL) {
        return -ENOMEM;
    }

    len = construct_frame(msg, buffer, len + 1);

    if (len < 0) {
        k_free(buffer);
        return len;
    }

    comms_write(comms, buffer, len);
    k_free(buffer);

    return 0;
}

int comms_write_msg(const struct comms *comms, const struct beluga_msg *msg) {
    int ret;
    if (comms == NULL || msg == NULL) {
        return -EINVAL;
    }

    switch (comms->ctx->format) {
    case FORMAT_ASCII:
    case FORMAT_JSON: {
        ret = comms_write_normal(comms, msg);
        break;
    }
    case FORMAT_FRAMES: {
        ret = comms_write_frame(comms, msg);
        break;
    }
    default: {
        ret = -EFAULT;
        break;
    }
    }

    return ret;
}

int print_format(const struct comms *comms) {
    struct beluga_msg msg = {.type = START_EVENT};
    int ret = 0;

    if (comms == NULL || comms->ctx == NULL) {
        return -EINVAL;
    }

    switch (comms->ctx->format) {
    case FORMAT_ASCII: {
        msg.payload.node_version = "  Format: ASCII";
        break;
    }
    case FORMAT_JSON: {
        msg.payload.node_version = "  Format: JSON";
        break;
    }
    case FORMAT_FRAMES: {
        msg.payload.node_version = "  Format: Framed";
        break;
    }
    default: {
        ret = -EFAULT;
        break;
    }
    }

    if (ret == 0) {
        ret = comms_write_msg(comms, &msg);
    }

    return ret;
}

int wait_comms_ready(const struct comms *comms) {
    if (comms == NULL || comms->ctx == NULL || comms->iface == NULL || comms->iface->api == NULL) {
        return -EINVAL;
    }
    if (comms->iface->api->wait_dtr == NULL) {
        return -ENOTSUP;
    }
    _COMMS_API(comms, wait_dtr);
    return 0;
}
