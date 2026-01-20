/**
 * @file comms.c
 *
 * @brief Front end interface for communicating over a serial bus.
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
#include <zephyr/logging/log.h>
#include <zephyr/sys/cbprintf.h>

/**
 * Logger for the comms module
 */
LOG_MODULE_REGISTER(comms_logger, CONFIG_COMMS_LOG_LEVEL);

/**
 * Private helper macro for calling API functions
 *
 * @param[in] _comms The comms object to fetch the API from
 * @param[in] _func The API function to call
 * @param[in] __VA_ARGS__ Additional arguments for the API function.
 */
#define _COMMS_API(_comms, _func, ...)                                         \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__),                                         \
                (comms->iface->api->_func(_comms->iface)),                     \
                (comms->iface->api->_func(_comms->iface, __VA_ARGS__)))

/**
 * Appends a byte to the receive buffer of the comms object
 *
 * @param[in] _comms The comms object to append the byte to
 * @param[in] data The byte to append
 */
#define _COMMS_BUF_APPEND(_comms, data)                                        \
    do {                                                                       \
        (_comms)->ctx->rx_buf.buf[(_comms)->ctx->rx_buf.len] = data;           \
        (_comms)->ctx->rx_buf.len++;                                           \
        (_comms)->ctx->rx_buf.buf[(_comms)->ctx->rx_buf.len] = '\0';           \
        (_comms)->ctx->rx_buf.len %= CONFIG_COMMS_RTX_BUF_SIZE;                \
    } while (0)

/**
 * The maximum amount of tokens an input string can be split into
 */
#define MAX_TOKENS CONFIG_COMMS_MAX_TOKENS

/**
 * Signature for a signal handler
 */
typedef void (*comms_signal_handler_t)(const struct comms *comms);

/**
 * @brief Generic signal handler that checks if the signal is set and runs
 * the handler if it is.
 *
 * @param[in] comms The comms object
 * @param[in] sig_idx The signal to check
 * @param[in] handler The handler to call if the signal is set
 */
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

/**
 * @brief Retrieves a command entry given an index
 * @param[in] id The index of the command entry
 * @return The command entry
 */
static inline const union at_command_entry *root_cmd_get(uint32_t id) {
    const union at_command_entry *cmd;

    TYPE_SECTION_GET(union at_command_entry, shell_root_cmds, id, &cmd);

    return cmd;
}

/**
 * @brief Retrieves the number of command entries in the commands section
 * @return The number of command entries
 */
static inline size_t root_cmd_count(void) {
    size_t len;

    TYPE_SECTION_COUNT(union at_command_entry, shell_root_cmds, &len);

    return len;
}

/**
 * @brief Retrieve an AT command entry given an index
 * @param[in] idx The index of the AT command to fetch
 * @return The AT command entry if successful
 * @return NULL if the index is outside the bounds of the commands section
 */
static inline const struct at_command_static_entry *at_cmd_get(size_t idx) {
    return (idx < root_cmd_count()) ? root_cmd_get(idx)->entry : NULL;
}

/**
 * @brief Retrieves an AT command entry based on the command string
 * @param[in] cmd_str The command string associated with the intended entry
 * @return The command entry if found
 * @return NULL if the command is not found
 */
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

/**
 * @brief Execute a command if an entry exists
 * @param[in] comms The comms object for the serial interface
 * @param[in] argc The number of tokens
 * @param[in] argv The arguments for the AT command
 * @param[in] entry The command entry
 * @return The return value of the command
 * @return -ENOTSUP if the command is not implemented
 */
static int execute_command(const struct comms *comms, size_t argc,
                           const char **argv,
                           const struct at_command_static_entry *entry) {

    if (entry->handler == NULL) {
        at_msg(comms, "Not implemented");
        return -ENOTSUP;
    }
    return entry->handler(comms, argc, argv);
}

/**
 * @brief Processes the input buffer and responds appropriately
 * @param[in] comms The comms object
 * @return The return code of the command
 * @return -ENOEXEC if command is not formatted correctly or does not exist
 * @return -ENOTSUP if command exists, but is not implemented
 */
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

/**
 * Thread for handling the communications
 * @param[in] comms_handle The comms handle associated with the serial bus being
 * used
 * @param[in] p2 unused
 * @param[in] p3 unused
 */
void comms_thread(void *comms_handle, void *p2, void *p3) {
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    struct comms *comms = comms_handle;
    int err;

    if (spawn_task_watchdog(&comms->ctx->watchdog) < 0) {
        LOG_ERR("Unable to spawn task watchdog in command thread\n");
        k_thread_abort(k_current_get());
    }

    if (pause_watchdog(&comms->ctx->watchdog) < 0) {
        LOG_ERR("Unable to pause task watchdog in command thread\n");
    }

    err = _COMMS_API(comms, enable, false);
    if (err != 0) {
        k_thread_abort(k_current_get());
    }

    while (true) {
        err = k_poll(comms->ctx->events, COMMS_SIGNAL_TXDONE, K_FOREVER);

        if (err != 0) {
            k_mutex_lock(&comms->ctx->wr_mtx, K_FOREVER);
            LOG_ERR("Comms thread error: %d", err);
            k_mutex_unlock(&comms->ctx->wr_mtx);
            k_thread_abort(k_current_get());
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

/**
 * @brief Raises a transport event
 * @param[in] evt_type The event to raise
 * @param[in] ctx The pointer to the comms object
 */
static void transport_evt_handler(enum comms_transport_evt evt_type,
                                  void *ctx) {
    struct comms *comms = ctx;
    struct k_poll_signal *signal;

    signal = (evt_type == COMMS_TRANSPORT_EVT_RX_RDY)
                 ? &comms->ctx->signals[COMMS_SIGNAL_RXRDY]
                 : &comms->ctx->signals[COMMS_SIGNAL_TXDONE];
    k_poll_signal_raise(signal, 0);
}

/**
 * @brief Initializes a comms instance
 * @param[in] comms The comms instance to instantiate
 * @param[in] transport_config Input parameter for the transport API init
 * @return The result of the API init
 */
static int instance_init(const struct comms *comms,
                         const void *transport_config) {
    struct task_wdt_attr watchdog = TASK_WDT_INITIALIZER(5000, comms->name);
    memset(comms->ctx, 0, sizeof(*comms->ctx));

    k_mutex_init(&comms->ctx->wr_mtx);

    for (size_t i = 0; i < COMMS_SIGNALS; i++) {
        k_poll_signal_init(&comms->ctx->signals[i]);
        k_poll_event_init(&comms->ctx->events[i], K_POLL_TYPE_SIGNAL,
                          K_POLL_MODE_NOTIFY_ONLY, &comms->ctx->signals[i]);
    }

    comms->ctx->watchdog = watchdog;

    return _COMMS_API(comms, init, transport_config, transport_evt_handler,
                      (void *)comms);
}

/**
 * @brief Function for initializing a transport layer and internal comms state.
 *
 * @param[in] comms		Pointer to comms instance.
 * @param[in] transport_config	Transport configuration during initialization.
 *
 * @return Standard error code.
 */
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

    k_tid_t tid =
        k_thread_create(comms->thread, comms->stack, CONFIG_COMMANDS_STACK_SIZE,
                        comms_thread, (void *)comms, NULL, NULL,
                        CONFIG_BELUGA_COMMANDS_PRIO, K_ESSENTIAL, K_NO_WAIT);
    comms->ctx->tid = tid;
    k_thread_name_set(tid, comms->name);

    return 0;
}

static void at_respond(const struct comms *comms, bool ok);

/**
 * @brief Process function, which should be executed when data is ready in the
 *	  transport interface.
 *
 * @param[in] comms Pointer to the comms instance.
 */
void comms_process(const struct comms *comms) {
    __ASSERT_NO_MSG(comms);
    __ASSERT_NO_MSG(comms->ctx);

    size_t count = 0;
    char data;
    struct comms_buf *buf = &comms->ctx->rx_buf;

    while (true) {
        watchdog_red_rocket(&comms->ctx->watchdog);
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
    if (pause_watchdog(&comms->ctx->watchdog) < 0) {
        LOG_ERR("Unable to pause task watchdog in command thread\n");
    }
}

/**
 * @brief Writes data to the comms transport API
 * @param[in] comms The comms object
 * @param[in] data The data to write
 * @param[in] length The number of bytes to write
 */
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

/**
 * @brief Appends a response ending to the TX buffer
 * @param[in] comms The comms object
 * @param[in] ok Indicator for successful command execution
 */
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

/**
 * @brief Copy a command response into the output buffer.
 *
 * @param[in] comms Pointer to the comms instance.
 * @param[in] msg The command response.
 *
 * @note This does not start the transmission of data.
 */
void at_msg(const struct comms *comms, const char *msg) {
    char *buf = comms->ctx->tx_buf.buf;
    size_t msg_len = comms->ctx->tx_buf.len;
    const size_t buf_size =
        sizeof(comms->ctx->tx_buf.buf) - 6; // Leave room for " OK\r\n"
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

/**
 * @brief Callback function for appending formatted strings
 * @param[in] c The byte to append
 * @param[in] ctx Additional context
 * @return always 0
 */
static int out_func(int c, void *ctx) {
    const struct comms *comms = ctx;
    char c_str[2] = {(char)c, '\0'};
    at_msg(comms, c_str);
    return 0;
}

/**
 * @brief Copy a formatted command response into the output buffer.
 * @param[in] comms Pointer to the comms instance
 * @param[in] msg The format string
 * @param[in] ... List of parameters to print
 *
 * @note This does not start the transmission of data.
 */
void at_msg_fmt(const struct comms *comms, const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    (void)cbvprintf(out_func, (void *)comms, msg, args);
    va_end(args);
}

/**
 * @brief Flushes the response buffer and blocks until everything has been
 * transmitted.
 * @param[in] comms The comms object
 * @param[in] ret The return code of the command
 */
void comms_flush_out(const struct comms *comms, int ret) {
    int set;
    int res;
    struct k_poll_signal *sig = &comms->ctx->signals[COMMS_SIGNAL_TXDONE];
    k_poll_signal_reset(sig);
    at_respond(comms, ret == 0);

    k_poll(&comms->ctx->events[COMMS_SIGNAL_TXDONE], 1, K_MSEC(500));
    k_poll_signal_check(sig, &set, &res);
}

/**
 * @brief Sets the output format
 * @param[in] comms The comms object
 * @param[in] mode The new mode
 * @return 0 upoin success
 * @return -EINVAL on invalid input parameters
 */
int set_format(const struct comms *comms, enum comms_out_format_mode mode) {
    if (mode >= FORMAT_INVALID || comms == NULL) {
        return -EINVAL;
    }
    comms->ctx->format = mode;
    return 0;
}

int set_wait_usb_host(const struct comms *comms, bool block) {
    if (comms == NULL) {
        return -EINVAL;
    }
    _COMMS_API(comms, block_no_usb_host, block);
    return 0;
}

bool comms_check_rx_error(const struct comms *comms) {
    if (comms == NULL) {
        return -EINVAL;
    }
    return _COMMS_API(comms, rx_error);
}

/**
 * @brief Generates the header output for ASCII mode
 *
 * @param[in] _comms The comms object
 * @param[in] _msg The header
 */
#define HEADER_GEN(_comms, _msg)                                               \
    do {                                                                       \
        const char *header = _msg;                                             \
        size_t header_len = sizeof((uint8_t[]){_msg}) - 1;                     \
        comms_write(_comms, header, header_len);                               \
    } while (0)

#if defined(CONFIG_UWB_LOGIC_CLK)
/**
 * The header for ASCII mode
 */
#define HEADER "ID,RSSI,RANGE,TIMESTAMP,EXCHANGE\r\n"

/**
 * The format string for ascii mode
 */
#define ASCII_FMT_STR "%" PRIu16 ",%" PRId8 ",%f,%" PRId64 ",%" PRIu32 "\r\n"

/**
 * The format string for JSON mode
 */
#define JSON_FMT_STR                                                           \
    "{ID:%" PRIu16 ",RSSI:%" PRId8 ",RANGE:%f,TIMESTAMP:%" PRId64              \
    ",EXCHANGE:%" PRIu32 "}\r\n"

/**
 * The format parameters
 *
 * @param[in] _i The neighbor list index
 */
#define FMT_PARAMS(_i)                                                         \
    list[(_i)].UUID, list[(_i)].RSSI, (double)list[(_i)].range,                \
        list[(_i)].time_stamp, list[(_i)].exchange_id
#else
/**
 * The header for ASCII mode
 */
#define HEADER        "ID,RSSI,RANGE,TIMESTAMP\r\n"

/**
 * The format string for ascii mode
 */
#define ASCII_FMT_STR "%" PRIu16 ",%" PRId8 ",%f,%" PRId64 "\r\n"

/**
 * The format string for JSON mode
 */
#define JSON_FMT_STR                                                           \
    "{ID:%" PRIu16 ",RSSI:%" PRId8 ",RANGE:%f,TIMESTAMP:%" PRId64 "}\r\n"

/**
 * The format parameters
 *
 * @param[in] _i The neighbor list index
 */
#define FMT_PARAMS(_i)                                                         \
    list[(_i)].UUID, list[(_i)].RSSI, (double)list[(_i)].range,                \
        list[(_i)].time_stamp
#endif

/**
 * @brief Formats and writes the neighbor list to the transport
 * @param[in] comms The comms object
 * @param[in] msg The beluga message to write
 * @return 0 upon success
 * @return -EINVAL if list is not present
 */
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

/**
 * @brief Writes a start event to the transport in ascii format
 * @param[in] comms The comms object
 * @param[in] msg The beluga message containing the start event
 * @return -EINVAL if payload is empty
 * @return 0 upon success
 */
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

/**
 * @brief Writes an error log to the transport in ascii format.
 * @param[in] comms The comms object.
 * @param[in] msg The beluga message containing the error message.
 * @return 0 upon success.
 * @return -EINVAL if there is no error message present.
 */
static int write_fatal_error(const struct comms *comms,
                             const struct beluga_msg *msg) {
    const char *ending = "\r\n";
    const char *preamble = "FATAL ERROR: ";
    const char *payload = msg->payload.error_message;

    if (!payload) {
        return -EINVAL;
    }

    comms_write(comms, preamble, 13);

    for (; *payload != '\0'; payload++) {
        comms_write(comms, payload, 1);
    }

    comms_write(comms, ending, 2);
    return 0;
}

/**
 * @brief Writes a dropped neighbor message to the transport
 * @param[in] comms The comms object
 * @param[in] msg The beluga message indicating which neighbor was dropped
 * @return 0 upon success
 * @return -ENOTSUP if not in JSON mode
 */
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

/**
 * @brief Write a message to the transport while in ASCII or JSON mode
 * @param[in] comms The comms object
 * @param[in] msg The message to format and write
 * @return 0 upon success
 * @return -ECANCELED if message type is not recognized
 * @return negative error code otherwise
 */
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
    case LOG_FATAL_ERROR: {
        ret = write_fatal_error(comms, msg);
        break;
    }
    default: {
        ret = -ECANCELED;
        break;
    }
    }

    return ret;
}

/**
 * @brief Writes a framed message over the comms transport
 * @param[in] comms The comms object
 * @param[in] msg The message to be framed and sent
 * @return 0 upon success
 * @return -EINVAL if frame length is too small
 * @return -ENOMEM if no dynamic memory is available
 */
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

/**
 * @brief Writes a message over the comms transport
 * @param[in] comms The comms object
 * @param[in] msg The message to write
 * @return 0 upon success
 * @return -EINVAL if parameters are invalid
 * @return -EFAULT if format mode is invalid
 * @return negative error code otherwise
 */
int comms_write_msg(const struct comms *comms, const struct beluga_msg *msg) {
    int ret;
    bool prev_state = false, fatal = false;
    if (comms == NULL || msg == NULL) {
        return -EINVAL;
    }

    if (msg->type == LOG_FATAL_ERROR) {
        struct comms_uart_common *ctx =
            (struct comms_uart_common *)comms->iface->ctx;
        prev_state = ctx->blocking_tx;
        ctx->blocking_tx = true;
        fatal = true;
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

    if (fatal) {
        struct comms_uart_common *ctx =
            (struct comms_uart_common *)comms->iface->ctx;
        ctx->blocking_tx = prev_state;
    }

    return ret;
}

/**
 * @brief Writes the current format mode over the transport
 * @param[in] comms The comms object
 * @return 0 upon success
 * @return -EINVAL if input parameters are incorrect
 * @return -EFAULT if invalid format mode
 * @return negative error code otherwise
 */
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

/**
 * @brief Sleeps the calling thread until the comms transport is ready
 * @param[in] comms The comms object
 * @return 0 upon success
 * @return -EINVAL if input is invalid
 * @return -ENOTSUP if transport does not have flow control
 */
int wait_comms_ready(const struct comms *comms) {
    if (comms == NULL || comms->ctx == NULL || comms->iface == NULL ||
        comms->iface->api == NULL) {
        return -EINVAL;
    }
    if (comms->iface->api->wait_dtr == NULL) {
        return -ENOTSUP;
    }
    _COMMS_API(comms, wait_dtr);
    return 0;
}

/**
 * @brief Determines if AT commands should respond with a simple OK or with
 * additional context
 * @param[in] comms The comms object
 * @param[in] verbose Determines the verbosity of the responses
 * @return 0 upon success
 * @return -EINVAL if input is invalid
 */
int set_verbosity(const struct comms *comms, bool verbose) {
    if (comms == NULL) {
        return -EINVAL;
    }
    comms->ctx->verbose = verbose;
    return 0;
}

void starve_comms_wdt(const struct comms *comms) {
    let_the_dog_starve(&comms->ctx->watchdog);
}
