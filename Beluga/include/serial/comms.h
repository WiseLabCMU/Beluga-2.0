/**
 * @file comms.h
 *
 * @brief Front end interface for communicating over a serial bus.
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

/**
 * Transport events
 */
enum comms_transport_evt {
    COMMS_TRANSPORT_EVT_RX_RDY, ///< Receiver ready
    COMMS_TRANSPORT_EVT_TX_RDY, ///< Transmitter ready
};

typedef void (*comms_transport_handler_t)(enum comms_transport_evt evt,
                                          void *context);

// Do not move this include
#include <serial/comms_uart.h>

struct at_command_static_entry;

/**
 * @brief AT command descriptor
 */
union at_command_entry {
    /** Pointer to array of static commands. */
    const struct at_command_static_entry *entry;
};

struct comms;

/**
 * @brief AT command handler prototype
 *
 * @param[in] comms Comms instance
 * @param[in] argc Argument count
 * @param[in] argv Arguments
 *
 * @return 0 Successful command execution
 * @return otherwise command execution failed
 */
typedef int (*at_command_handler)(const struct comms *comms, size_t argc,
                                  char const *const *argv);

/**
 * @brief AT command static command descriptor
 */
struct at_command_static_entry {
    const char *command;        ///< Command name
    at_command_handler handler; ///< Command handler
};

/**
 * Macro for defining an AT command entry
 *
 * @param[in] _command The name of the AT command
 */
#define AT_CMD_DEFINE(_command)                                                \
    __attribute__((unused)) static int at_##_command(                          \
        const struct comms *comms, size_t argc, char const *const *argv)

/**
 * @brief Macro for defining and adding an AT command.
 *
 * @note Each command shall have a unique name.
 *
 * @param[in] _command The name of the AT command (for example: TIME).
 */
#define AT_CMD_REGISTER(_command)                                              \
    static const struct at_command_static_entry UTIL_CAT(_comms_,              \
                                                         _command) = {         \
        .command = (const char *)STRINGIFY(_command),                          \
        .handler = (at_command_handler)UTIL_CAT(at_, _command),                \
    };                                                                         \
    static const TYPE_SECTION_ITERABLE(                                        \
        union at_command_entry, UTIL_CAT(at_cmd_, _command), shell_root_cmds,  \
        UTIL_CAT(at_cmd_, _command)) = {.entry = &UTIL_CAT(_comms_, _command)}

/**
 * @brief Macro for defining and adding a conditional AT command.
 *
 * @see AT_CMD_REGISTER for details.
 *
 * Macro can be used to create a command which can be conditionally present.
 * It is an alternative to \#ifdefs around the command registration and command
 * handler. If command is disabled, the handler is removed from the application.
 *
 * @param[in] _flag Compile time flag. Command is present only if the flag
 * exists and equals to 1.
 * @param[in] _command The name of the AT command (for example: TIME).
 */
#define AT_CMD_COND_REGISTER(_flag, _command)                                  \
    IF_ENABLED(_flag, (AT_CMD_REGISTER(_command)))

/**
 * Cannot use with shell
 */
BUILD_ASSERT(!IS_ENABLED(CONFIG_SHELL),
             "Shell cannot be enabled when using AT commands");

struct comms_transport;

/**
 * @struct comms_transport_api
 * @brief Unified comms transport interface
 */
struct comms_transport_api {
    /**
     * @brief Function for initializing the comms transport interface.
     *
     * @param[in] transport   Pointer to the transfer instance.
     * @param[in] config      Pointer to instance configuration.
     * @param[in] evt_handler Event handler.
     * @param[in] context     Pointer to the context passed to event
     *			  handler.
     *
     * @return Standard error code.
     */
    int (*init)(const struct comms_transport *transport, const void *config,
                comms_transport_handler_t evt_handler, void *context);

    /**
     * @brief Function for uninitializing the comms transport interface.
     *
     * @param[in] transport  Pointer to the transfer instance.
     *
     * @return Standard error code.
     */
    int (*uninit)(const struct comms_transport *transport);

    /**
     * @brief Function for enabling transport in given TX mode.
     *
     * Function can be used to reconfigure TX to work in blocking mode.
     *
     * @param transport   Pointer to the transfer instance.
     * @param blocking_tx If true, the transport TX is enabled in blocking
     *		      mode.
     *
     * @return NRF_SUCCESS on successful enabling, error otherwise (also if
     * not supported).
     */
    int (*enable)(const struct comms_transport *transport, bool blocking_tx);

    /**
     * @brief Function for writing data to the transport interface.
     *
     * @param[in]  transport  Pointer to the transfer instance.
     * @param[in]  data       Pointer to the source buffer.
     * @param[in]  length     Source buffer length.
     * @param[out] cnt        Pointer to the sent bytes counter.
     *
     * @return Standard error code.
     */
    int (*write)(const struct comms_transport *transport, const void *data,
                 size_t length, size_t *cnt);

    /**
     * @brief Function for reading data from the transport interface.
     *
     * @param[in]  transport    Pointer to the transfer instance.
     * @param[in]  data         Pointer to the destination buffer.
     * @param[in]  length       Destination buffer length.
     * @param[out] cnt          Pointer to the received bytes counter.
     *
     * @return Standard error code.
     */
    int (*read)(const struct comms_transport *transport, void *data,
                size_t length, size_t *cnt);

    /**
     * @brief Function called in comms thread loop.
     *
     * Can be used for backend operations that require longer execution time
     *
     * @param[in] transport Pointer to the transfer instance.
     */
    void (*update)(const struct comms_transport *transport);

    /**
     * @brief Function that blocks execution until the DTR control line
     * is asserted.
     *
     * @param[in] transport Pointer to the transfer instance
     */
    void (*wait_dtr)(const struct comms_transport *transport);
};

/**
 * @struct comms_transport
 * @brief Container for the API and additional context
 */
struct comms_transport {
    const struct comms_transport_api *api;
    void *ctx;
};

/**
 * @brief Signals for the comms interface
 */
enum comms_signal {
    COMMS_SIGNAL_RXRDY,  ///< RX data ready
    COMMS_SIGNAL_TXDONE, ///< DOne transmitting. Must be last one before
                         ///< COMMS_SIGNALS
    COMMS_SIGNALS        ///< Last enumerator
};

/**
 * @brief Output format mode
 *
 * - FORMAT_ASCII:
 *  When in ASCII mode, flashed settings are printed along with node version
 *  at the start of the application. The neighbor list is printed with a header
 *  in each cycle, and the collected data is printed in a CSV format. Command
 *  responses are printed as raw ASCII strings.
 *
 * - FORMAT_JSON:
 *  When in JSON mode, flashed settings are printed along with node version at
 * the start of the application, like in ASCII mode. The neighbor list is
 * printed in JSON format, and removed neighbors are indicated with "rm <node
 * ID>." Command responses are printed as raw ASCII strings.
 *
 * - FORMAT_FRAMES:
 *  When in framed mode, only the node version is printed at the start of the
 * program with the START_EVENT frame. The neighbor list is printed as a JSON
 * list with the NEIGHBOR_UPDATE frame. Neighbor removals are printed as a node
 * ID with the NEIGHBOR_DROP frame. Successful UWB ranging responses are printed
 * as a JSON with the RANGING_EVENT frame. Command responses are printed with
 * the COMMAND_RESPONSE frame. Settings saved in flash must be retrieved through
 * commands.
 *
 * @note There are 5 things that can be printed: Command responses, the neighbor
 * list, successful ranging responses, neighbor removals, and boot events. If it
 * is not mentioned, then it is not printed in that mode.
 */
enum comms_out_format_mode {
    FORMAT_ASCII = 0,  ///< ASCII mode
    FORMAT_JSON = 1,   ///< JSON mode
    FORMAT_FRAMES = 2, ///< Framed mode
    FORMAT_INVALID     ///< Last output format (Does nothing)
};

/**
 * @struct comms_buf
 * @brief Unified buffer structure. Stores buffer and the number of valid
 * bytes starting from index 0.
 */
struct comms_buf {
    uint8_t buf[CONFIG_COMMS_RTX_BUF_SIZE + 1];
    size_t len;
};

/**
 * @brief Comms instance context
 */
struct comms_ctx {
    struct comms_buf rx_buf; ///< Receive buffer
    struct comms_buf tx_buf; ///< Transmit buffer

    /**
     * Signals for raising events
     */
    struct k_poll_signal signals[COMMS_SIGNALS];

    /**
     * Events that should be used only internally by the comms
     * thread.
     */
    struct k_poll_event events[COMMS_SIGNALS];

    struct k_mutex wr_mtx;             ///< Write lock
    k_tid_t tid;                       ///< Comms thread ID
    enum comms_out_format_mode format; ///< Output mode
    bool verbose;                      ///< Verbose mode
};

/**
 * @brief Comms instance internals
 */
struct comms {
    const struct comms_transport *iface; ///< Transport interface
    struct comms_ctx *ctx;               ///< Internal context
    const char *name;
    struct k_thread *thread;
    k_thread_stack_t *stack;
};

/**
 * @brief Macro for defining a comms instance.
 *
 * @param[in] _name Instance name.
 * @param[in] _transport Pointer to the transport interface.
 */
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

/**
 * @brief Function for initializing a transport layer and internal comms state.
 *
 * @param[in] sh		Pointer to comms instance.
 * @param[in] transport_config	Transport configuration during initialization.
 *
 * @return Standard error code.
 */
int comms_init(const struct comms *comms, const void *transport_config);

/**
 * @brief Process function, which should be executed when data is ready in the
 *	  transport interface.
 *
 * @param[in] sh Pointer to the comms instance.
 */
void comms_process(const struct comms *comms);

/**
 * @brief Copy a command response into the output buffer.
 *
 * @param[in] comms Pointer to the comms instance.
 * @param[in] msg The command response.
 *
 * @note This does not start the transmission of data.
 */
void at_msg(const struct comms *comms, const char *msg);

/**
 * @brief Copy a formatted command response into the output buffer.
 * @param[in] comms Pointer to the comms instance
 * @param[in] msg The format string
 * @param[in] ... List of parameters to print
 *
 * @note This does not start the transmission of data.
 */
void __printflike(2, 3)
    at_msg_fmt(const struct comms *comms, const char *msg, ...);

/**
 * @brief Sets the output format
 * @param[in] comms The comms object
 * @param[in] mode The new mode
 * @return 0 upoin success
 * @return -EINVAL on invalid input parameters
 */
int set_format(const struct comms *comms, enum comms_out_format_mode mode);

/**
 * @brief Writes the current format mode over the transport
 * @param[in] comms The comms object
 * @return 0 upon success
 * @return -EINVAL if input parameters are incorrect
 * @return -EFAULT if invalid format mode
 * @return negative error code otherwise
 */
int print_format(const struct comms *comms);

/**
 * @brief Writes a message over the comms transport
 * @param[in] comms The comms object
 * @param[in] msg The message to write
 * @return 0 upon success
 * @return -EINVAL if parameters are invalid
 * @return -EFAULT if format mode is invalid
 * @return negative error code otherwise
 */
int comms_write_msg(const struct comms *comms, const struct beluga_msg *msg);

/**
 * @brief Flushes the response buffer and blocks until everything has been
 * transmitted.
 * @param[in] comms The comms object
 * @param[in] ret The return code of the command
 */
void comms_flush_out(const struct comms *comms, int ret);

/**
 * @brief Sleeps the calling thread until the comms transport is ready
 * @param[in] comms The comms object
 * @return 0 upon success
 * @return -EINVAL if input is invalid
 * @return -ENOTSUP if transport does not have flow control
 */
int wait_comms_ready(const struct comms *comms);

/**
 * @brief Determines if AT commands should respond with a simple OK or with
 * additional context
 * @param[in] comms The comms object
 * @param[in] verbose Determines the verbosity of the responses
 * @return 0 upon success
 * @return -EINVAL if input is invalid
 */
int set_verbosity(const struct comms *comms, bool verbose);

/**
 * @brief Helper that prints an AT command response and indicates that the
 * command executed correctly.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _msg The command response message
 */
#define Z_OK_MSG_NOARGS(_comms, _msg)                                          \
    do {                                                                       \
        at_msg(_comms, _msg);                                                  \
        return 0;                                                              \
    } while (0)

/**
 * @brief Helper that prints an AT command response and indicates that the
 * command executed correctly.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _msg The command response format string
 * @param[in] ... List of parameters to print
 */
#define Z_OK_MSG_ARGS(_comms, _msg, ...)                                       \
    do {                                                                       \
        at_msg_fmt(_comms, _msg, __VA_ARGS__);                                 \
        return 0;                                                              \
    } while (0)

/**
 * @brief Helper that prints an AT command response and indicates that the
 * command executed correctly.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _msg The command response format string
 * @param[in] ... Optional list of parameters to print
 */
#define Z_OK_MSG(_comms, _msg, ...)                                            \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (Z_OK_MSG_NOARGS(_comms, _msg)),        \
                (Z_OK_MSG_ARGS(_comms, _msg, __VA_ARGS__)))

/**
 * @brief Macro that indicates that an AT command executed correctly.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] ... Optional message to print with an "OK" response
 *
 * @note If the command should change its response based on the verbosity
 * flag, then use `AT_OK`
 */
#define OK(_comms, ...)                                                        \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (return 0),                             \
                (Z_OK_MSG(_comms, GET_ARG_N(1, __VA_ARGS__),                   \
                          GET_ARGS_LESS_N(1, __VA_ARGS__))))

/**
 * @brief Macro that indicates that an AT command executed correctly.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] msg_ The verbose message
 * @param[in] ... Optional message format parameters
 */
#define AT_OK(_comms, msg_, ...)                                               \
    do {                                                                       \
        if ((_comms)->ctx->verbose) {                                          \
            OK(_comms, msg_, __VA_ARGS__);                                     \
        } else {                                                               \
            OK(_comms);                                                        \
        }                                                                      \
    } while (0)

/**
 * @brief Helper macro that inserts an AT command response into the transmit
 * buffer without returning success or failure.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _msg The command response message
 */
#define Z_NOW_MSG_NOARGS(_comms, _msg) at_msg(_comms, _msg)

/**
 * @brief Helper macro that inserts a formatted AT command response into the
 * transmit buffer without returning success or failure.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _msg The command response message
 * @param[in] ... List of parameters to print
 */
#define Z_NOW_MSG_ARGS(_comms, _msg, ...) at_msg_fmt(_comms, _msg, __VA_ARGS__)

/**
 * @brief Helper macro that inserts an AT command response into the
 * transmit buffer without returning success or failure.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _msg The command response message
 * @param[in] ... Optional list of parameters to print
 */
#define Z_NOW_MSG(_comms, _msg, ...)                                           \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (Z_NOW_MSG_NOARGS(_comms, _msg)),       \
                (Z_NOW_MSG_ARGS(_comms, _msg, __VA_ARGS__)))

/**
 * @brief Helper macro that inserts an AT command response into the transmit
 * buffer if there is one present without returning success or failure.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] ... Optional message and list of parameters to print
 */
#define Z_INSERT_MSG_NOW(_comms, ...)                                          \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (),                                     \
                (Z_NOW_MSG(_comms, GET_ARG_N(1, __VA_ARGS__),                  \
                           GET_ARGS_LESS_N(1, __VA_ARGS__));))

/**
 * @brief Macro that inserts an AT command response into the transmit
 * buffer. Additionally, this macro will transmit the response before the
 * command returns. This is best used in cases where the command will
 * not return (for example, shutting down the device).
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] ... Optional message and list of parameters to print
 *
 * @note If the command should change its response based on the verbosity
 * flag, then use `AT_OK_NOW`
 */
#define OK_NOW(_comms, ...)                                                    \
    do {                                                                       \
        Z_INSERT_MSG_NOW(_comms, __VA_ARGS__) comms_flush_out(_comms, 0);      \
    } while (0)

/**
 * @brief Macro that inserts an AT command response into the transmit
 * buffer. Additionally, this macro will transmit the response before the
 * command returns. This is best used in cases where the command will
 * not return (for example, shutting down the device).
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] msg_ The verbose message
 * @param[in] ... Optional message and list of parameters to print
 */
#define AT_OK_NOW(_comms, msg_, ...)                                           \
    do {                                                                       \
        if ((_comms)->ctx->verbose) {                                          \
            OK_NOW(_comms, msg_, __VA_ARGS__);                                 \
        } else {                                                               \
            OK_NOW(_comms);                                                    \
        }                                                                      \
    } while (0)

/**
 * @brief Helper that prints an AT command response and indicates that the
 * command failed.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _msg The command response message
 */
#define Z_ERR_MSG_NOARGS(_comms, _msg)                                         \
    do {                                                                       \
        at_msg(_comms, _msg);                                                  \
        return 1;                                                              \
    } while (0)

/**
 * @brief Helper that prints an AT command response and indicates that the
 * command failed.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _msg The command response format string
 * @param[in] ... List of parameters to print
 */
#define Z_ERR_MSG_ARGS(_comms, _msg, ...)                                      \
    do {                                                                       \
        at_msg_fmt(_comms, _msg, __VA_ARGS__);                                 \
        return 1;                                                              \
    } while (0)

/**
 * @brief Macro that indicates that an AT command failed.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _msg Failure response message
 * @param[in] ... Optional list of print parameters
 */
#define ERROR(_comms, _msg, ...)                                               \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (Z_ERR_MSG_NOARGS(_comms, _msg)),       \
                (Z_ERR_MSG_ARGS(_comms, _msg, __VA_ARGS__)))

/**
 * @brief Helper macro that inserts an AT command response into the
 * transmit buffer without returning success or failure.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _msg The command response message
 */
#define Z_ERR_MSG_NOARGS_NORET(_comms, _msg) at_msg(_comms, _msg)

/**
 * @brief Helper macro that inserts an AT command response into the
 * transmit buffer without returning success or failure.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _msg The command response message
 * @param[in] ... Optional list of parameters to print
 */
#define Z_ERR_MSG_ARGS_NORET(_comms, _msg, ...)                                \
    at_msg_fmt(_comms, _msg, __VA_ARGS__)

/**
 * @brief Macro that inserts an AT command response into the transmit
 * buffer if there is one present without returning success or failure.
 * This is best used in cases where an error is non-fatal, but should be
 * printed early anyway.
 *
 * @param[in] _comms Pointer to the comms instance
 * @param[in] _msg The command response message
 * @param[in] ... Optional list of parameters to print
 */
#define ERROR_NORET(_comms, _msg, ...)                                         \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (Z_ERR_MSG_NOARGS_NORET(_comms, _msg)), \
                (Z_ERR_MSG_ARGS_NORET(_comms, _msg, __VA_ARGS__)))

#endif // BELUGA_COMMS_H
