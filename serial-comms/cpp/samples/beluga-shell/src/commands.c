/**
 * @file commands.c
 *
 * @brief
 *
 * @date 12/17/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <argp.h>
#include <autocomplete.h>
#include <commands.h>
#include <critical_sections.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <readline/readline.h>
#include <shell_helpers.h>
#include <sio.h>
#include <stdio.h>
#include <utils.h>

#define _COMMAND_DEFAULT_NAME(handler_)                                        \
    { #handler_, handler_ }
#define _COMMAND_CUSTOM_NAME(handler_, name_)                                  \
    { name_, handler_ }

#define COMMAND(handler_, name_...)                                            \
    COND_CODE_0(IS_EMPTY(name_), (_COMMAND_CUSTOM_NAME(handler_, name_)),      \
                (_COMMAND_DEFAULT_NAME(handler_)))

static void id(const struct cmdline_tokens *tokens);
static void start_ble(const struct cmdline_tokens *tokens);
static void stop_ble(const struct cmdline_tokens *tokens);
static void start_uwb(const struct cmdline_tokens *tokens);
static void stop_uwb(const struct cmdline_tokens *tokens);
static void bootmode(const struct cmdline_tokens *tokens);
static void rate(const struct cmdline_tokens *tokens);
static void channel(const struct cmdline_tokens *tokens);
static void reset(const struct cmdline_tokens *tokens);
static void timeout(const struct cmdline_tokens *tokens);
static void txpower(const struct cmdline_tokens *tokens);
static void streammode(const struct cmdline_tokens *tokens);
static void twrmode(const struct cmdline_tokens *tokens);
static void ledmode(const struct cmdline_tokens *tokens);
static void reboot(const struct cmdline_tokens *tokens);
static void pwramp(const struct cmdline_tokens *tokens);
static void antenna(const struct cmdline_tokens *tokens);
static void cmd_time(const struct cmdline_tokens *tokens);
static void deepsleep(const struct cmdline_tokens *tokens);
static void datarate(const struct cmdline_tokens *tokens);
static void preamble(const struct cmdline_tokens *tokens);
static void pulserate(const struct cmdline_tokens *tokens);
static void phr(const struct cmdline_tokens *tokens);
static void pac(const struct cmdline_tokens *tokens);
static void sfd(const struct cmdline_tokens *tokens);
static void panid(const struct cmdline_tokens *tokens);
static void evict(const struct cmdline_tokens *tokens);
static void verbose(const struct cmdline_tokens *tokens);
static void status(const struct cmdline_tokens *tokens);
static void version(const struct cmdline_tokens *tokens);
static void starve(const struct cmdline_tokens *tokens);
static void exchange(const struct cmdline_tokens *tokens);
static void reason(const struct cmdline_tokens *tokens);
static void jobs(const struct cmdline_tokens *tokens);
static void bg(const struct cmdline_tokens *tokens);
static void fg(const struct cmdline_tokens *tokens);
static void config_beluga_stream(const struct cmdline_tokens *tokens);
static void neighbors(const struct cmdline_tokens *tokens);

struct stream_ctx {
    int fd;
    char *filepath;
    pthread_mutex_t lock;
    bool streaming;
};

struct _serial_attr {
    struct beluga_serial *serial;
    struct stream_ctx error;
    struct stream_ctx exchange;
    struct stream_ctx ranges;
    struct stream_ctx neighbors;
    pthread_mutex_t serial_lock;
    bool expecting_reboot;
};

static struct _serial_attr attr = {
    .serial = NULL,
    .error =
        {
            .fd = STDOUT_FILENO,
            .streaming = true,
        },
    .exchange = {.fd = STDOUT_FILENO},
    .ranges = {.fd = STDOUT_FILENO},
    .neighbors = {.fd = STDOUT_FILENO},
};

static struct command_info commands[] = {
    COMMAND(id),
    COMMAND(start_ble, "start-ble"),
    COMMAND(stop_ble, "stop-ble"),
    COMMAND(start_uwb, "start-uwb"),
    COMMAND(stop_uwb, "stop-uwb"),
    COMMAND(bootmode),
    COMMAND(rate),
    COMMAND(channel),
    COMMAND(reset),
    COMMAND(timeout),
    COMMAND(txpower),
    COMMAND(streammode),
    COMMAND(twrmode),
    COMMAND(ledmode),
    COMMAND(reboot),
    COMMAND(pwramp),
    COMMAND(antenna),
    COMMAND(cmd_time, "time"),
    COMMAND(deepsleep),
    COMMAND(datarate),
    COMMAND(preamble),
    COMMAND(pulserate),
    COMMAND(phr),
    COMMAND(pac),
    COMMAND(sfd),
    COMMAND(panid),
    COMMAND(evict),
    COMMAND(verbose),
    COMMAND(status),
    COMMAND(version),
    COMMAND(starve),
    COMMAND(exchange),
    COMMAND(reason),
    COMMAND(jobs),
    COMMAND(bg),
    COMMAND(fg),
    COMMAND(config_beluga_stream, "manage-stream"),
    COMMAND(neighbors),
};

#define ARRAY_SIZE(array_) sizeof(array_) / sizeof(array_[0])

int initialize_builtin_commands(struct beluga_serial *serial) {
    if (serial == NULL) {
        return -EINVAL;
    }
    attr.serial = serial;
    pthread_mutex_init(&attr.error.lock, NULL);
    pthread_mutex_init(&attr.exchange.lock, NULL);
    pthread_mutex_init(&attr.neighbors.lock, NULL);
    pthread_mutex_init(&attr.ranges.lock, NULL);
    pthread_mutex_init(&attr.serial_lock, NULL);

    if (register_command_callbacks(commands, ARRAY_SIZE(commands)) < 0) {
        printf("Failed to initialize command handlers\n");
        exit(EXIT_FAILURE);
    }

    for (size_t i = 0; i < ARRAY_SIZE(commands); i++) {
        autocomplete_register_builtin_command(commands[i].cmd_str);
    }

    return 0;
}

void report_unexpected_reboot(void) {
    const char msg[] = "\nBeluga rebooted unexpectedly\n";

    MUTEX_CRITICAL_SECTION(&attr.error.lock) {
        int fd = attr.error.fd;

        if (!attr.error.streaming) {
            MUTEX_SECTION_BREAK;
        }

        ssize_t err = write(fd, msg, sizeof(msg) - 1);
        if (err < 0) {
            perror("open");
        }

        if (fd == STDOUT_FILENO) {
            rl_redraw_prompt_last_line();
        }
    }
}

void handle_resync(void) {
    char response[sizeof(attr.serial->response)];

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_reason(attr.serial);
        strcpy(response, attr.serial->response);
    }

    MUTEX_CRITICAL_SECTION(&attr.error.lock) {
        int fd = attr.error.fd;

        if (attr.expecting_reboot) {
            attr.expecting_reboot = false;
            MUTEX_SECTION_BREAK;
        }

        if (!attr.error.streaming) {
            MUTEX_SECTION_BREAK;
        }

        dprintf(fd, "\n%s\n", response);

        if (fd == STDOUT_FILENO) {
            rl_redraw_prompt_last_line();
        }
    }
}

void report_fatal_error(const char *msg) {
    MUTEX_CRITICAL_SECTION(&attr.error.lock) {
        int fd = attr.error.fd;

        if (fd == STDOUT_FILENO) {
            sio_dprintf(fd, "\n%s\n", msg);
            rl_redraw_prompt_last_line();
        } else {
            sio_dprintf(fd, "%s\n", msg);
        }
    }
}

void report_range_event(const struct range_event *event) {
    MUTEX_CRITICAL_SECTION(&attr.exchange.lock) {
        int fd = attr.exchange.fd;

        if (!attr.exchange.streaming) {
            MUTEX_SECTION_BREAK;
        }

        sio_dprintf(fd,
                    "\n-----\nExchange Event:\nID: %d\nExchange: "
                    "%d\nTimestamp: %ld\n-----\n",
                    event->id, event->exchange, event->timestamp);

        if (fd == STDOUT_FILENO) {
            rl_redraw_prompt_last_line();
        }
    }
}

void report_neighbor_update(const struct beluga_neighbor *updates, size_t len) {
    MUTEX_CRITICAL_SECTION(&attr.neighbors.lock) {
        int fd = attr.neighbors.fd;

        if (!attr.neighbors.streaming) {
            MUTEX_SECTION_BREAK;
        }

        sio_dprintf(fd, "\n-----\nNeighbor Event:\n");
        for (size_t i = 0; i < len; i++) {
            sio_dprintf(
                fd,
                "ID: %d\nRSSI: %d\nExchange: %d\nRange: %f\nTimestamp: %ld\n",
                updates[i].id, updates[i].rssi, updates[i].exchange,
                updates[i].range, updates[i].time);
            if ((i + 1) < len) {
                sio_dprintf(fd, "\n");
            }
        }
        sio_dprintf(fd, "-----\n");

        if (fd == STDOUT_FILENO) {
            rl_redraw_prompt_last_line();
        }
    }
}

void report_range_update(const struct beluga_neighbor *updates, size_t len) {
    MUTEX_CRITICAL_SECTION(&attr.ranges.lock) {
        int fd = attr.ranges.fd;

        if (!attr.ranges.streaming) {
            MUTEX_SECTION_BREAK;
        }

        sio_dprintf(fd, "\n-----\nRange Event:\n");
        for (size_t i = 0; i < len; i++) {
            sio_dprintf(
                fd,
                "ID: %d\nRSSI: %d\nExchange: %d\nRange: %f\nTimestamp: %ld\n",
                updates[i].id, updates[i].rssi, updates[i].exchange,
                updates[i].range, updates[i].time);
            if ((i + 1) < len) {
                sio_dprintf(fd, "\n");
            }
        }
        sio_dprintf(fd, "-----\n");

        if (fd == STDOUT_FILENO) {
            rl_redraw_prompt_last_line();
        }
    }
}

static void write_serial_response(const struct cmdline_tokens *tokens) {
    int output_fd = STDOUT_FILENO;

    if (tokens->outfile != NULL) {
        if (!tokens->outfile_append) {
            output_fd = open(tokens->outfile, WR_FLAGS, WR_PERMS);
        } else {
            output_fd = open(tokens->outfile, APPEND_FLAGS, WR_PERMS);
        }

        if (output_fd < 0) {
            perror(tokens->outfile);
            return;
        }
    }

    sio_dprintf(output_fd, "%s\n", attr.serial->response);
}

static void id(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_id(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void start_ble(const struct cmdline_tokens *tokens) {
    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_start_ble(attr.serial);
        write_serial_response(tokens);
    }
}

static void stop_ble(const struct cmdline_tokens *tokens) {
    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_stop_ble(attr.serial);
        write_serial_response(tokens);
    }
}

static void start_uwb(const struct cmdline_tokens *tokens) {
    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_start_uwb(attr.serial);
        write_serial_response(tokens);
    }
}

static void stop_uwb(const struct cmdline_tokens *tokens) {
    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_stop_uwb(attr.serial);
        write_serial_response(tokens);
    }
}

static void bootmode(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_bootmode(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void rate(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_rate(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void channel(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_channel(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void reset(const struct cmdline_tokens *tokens) {
    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_reset(attr.serial);
        write_serial_response(tokens);
    }
}

static void timeout(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_timeout(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static bool strtouint32(const char *str, uint32_t *result) {
    char *endptr;
    unsigned long ret;
    char *start = (char *)str;

    errno = 0;
    ret = strtoul(start, &endptr, 10);

    if (errno == ERANGE || (uint64_t)ret > (uint64_t)UINT32_MAX ||
        isgraph((int)*endptr)) {
        *result = 0;
        return false;
    }

    *result = (uint32_t)ret;
    return true;
}

static void handle_txpower1(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    if (tokens->argc == 3) {
        sio_printf("Invalid number of parameters\n");
        return;
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_txpower(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void handle_txpower2(const struct cmdline_tokens *tokens) {
    enum uwb_amp_stage stage;
    uint32_t coarse, fine;
    uint32_t _stage;

    if (!strtouint32(tokens->argv[1], &_stage)) {
        sio_printf("Stage argument must be a 32-bit unsigned integer\n");
        return;
    }

    switch (_stage) {
    case 0:
        stage = BOOST_NORM;
        break;
    case 1:
        stage = BOOSTP_500;
        break;
    case 2:
        stage = BOOSTP_250;
        break;
    case 3:
        stage = BOOSTP_125;
        break;
    default:
        sio_printf("Invalid stage ID. Must be 0, 1, 2, or 3\n");
        return;
    }

    if (!strtouint32(tokens->argv[2], &coarse)) {
        sio_printf("Coarse argument must be a 32-bit unsigned integer\n");
        return;
    }

    if (!strtouint32(tokens->argv[3], &fine)) {
        sio_printf("Fine argument must be a 32-bit unsigned integer\n");
        return;
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_txpower2(attr.serial, stage, coarse, fine);
        write_serial_response(tokens);
    }
}

static void txpower(const struct cmdline_tokens *tokens) {
    if (tokens->argc < 4) {
        handle_txpower1(tokens);
    } else {
        handle_txpower2(tokens);
    }
}

static void streammode(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_streammode(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void twrmode(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_twrmode(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void ledmode(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_ledmode(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void reboot(const struct cmdline_tokens *tokens) {
    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        attr.expecting_reboot = true;
        beluga_serial_reboot(attr.serial);
        write_serial_response(tokens);
    }
}

static void pwramp(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_pwramp(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void antenna(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_antenna(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void cmd_time(const struct cmdline_tokens *tokens) {
    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_time(attr.serial);
        write_serial_response(tokens);
    }
}

static void deepsleep(const struct cmdline_tokens *tokens) {
    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_deepsleep(attr.serial);
        write_serial_response(tokens);
    }
}

static void datarate(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_datarate(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void preamble(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_preamble(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void pulserate(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_pulserate(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void phr(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_phr(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void pac(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_pac(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void sfd(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_sfd(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void panid(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_panid(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void evict(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_evict(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void verbose(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_verbose(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void status(const struct cmdline_tokens *tokens) {
    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_status(attr.serial);
        write_serial_response(tokens);
    }
}

static void version(const struct cmdline_tokens *tokens) {
    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_version(attr.serial);
        write_serial_response(tokens);
    }
}

static void starve(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_starve(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void exchange(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_exchange(attr.serial, arg);
        write_serial_response(tokens);
    }
}

static void reason(const struct cmdline_tokens *tokens) {
    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        beluga_serial_reason(attr.serial);
        write_serial_response(tokens);
    }
}

static void jobs(const struct cmdline_tokens *tokens) {
    sigset_t mask, prev;
    int fd = STDOUT_FILENO;

    CRITICAL_SECTION(mask, prev) {
        if (tokens->outfile) {
            if (!tokens->outfile_append) {
                fd = open(tokens->outfile, WR_FLAGS, WR_PERMS);
            } else {
                fd = open(tokens->outfile, APPEND_FLAGS, WR_PERMS);
            }

            if (fd < 0) {
                perror(tokens->outfile);
                CRITICAL_SECTION_BREAK;
            }
        }

        if (!list_jobs(fd)) {
            sio_printf("Error printing out job list\n");
        }

        if (fd != STDOUT_FILENO) {
            close(fd);
        }
    }
}

static bool strtoint32(const char *str, int32_t *result) {
    char *endptr;
    unsigned long ret;
    char *start = (char *)str;

    errno = 0;
    ret = strtol(start, &endptr, 10);

    if (errno == ERANGE || (int64_t)ret > (int64_t)INT32_MAX ||
        (int64_t)ret < (int64_t)INT32_MIN || isgraph((int)*endptr)) {
        *result = 0;
        return false;
    }

    *result = (int32_t)ret;
    return true;
}

extern sig_atomic_t fg_running; // declared in main.c

static void update_job(jid_t job, pid_t pid, enum job_state state,
                       sigset_t *prev) {
    sio_assert(state == BG || state == FG);
    if (state == BG) {
        job_set_state(job, state);
        sio_printf("[%d] (%ld) %s\n", (int)job, (long)pid,
                   job_get_cmdline(job));
        return;
    }

    sio_assert(fg_job() == (jid_t)0);

    fg_running = 1;

    job_set_state(job, state);
    kill(-pid, SIGCONT);

    while (fg_running) {
        sigsuspend(prev);
    }
}

static void continue_job(const struct cmdline_tokens *tokens,
                         enum job_state state) {
    sigset_t mask, prev;
    jid_t job;
    pid_t pid;
    int32_t parsed_number;
    char *arg1;
    bool job_arg = false;

    if (tokens->argc == 1) {
        sio_eprintf("%s command requires PID or %%jobid argument\n",
                    tokens->argv[0]);
        return;
    }

    if (tokens->argv[1][0] == '%') {
        arg1 = tokens->argv[1] + 1;
        job_arg = true;
    } else {
        arg1 = tokens->argv[1];
    }

    if (!strtoint32(arg1, &parsed_number)) {
        sio_eprintf("%s: argument must be a PID or %%jobid\n", tokens->argv[0]);
        return;
    }

    CRITICAL_SECTION(mask, prev) {
        if (job_arg) {
            job = (jid_t)parsed_number;
            if (!job_exists(job)) {
                sio_eprintf("%s: No such job\n", tokens->argv[1]);
                CRITICAL_SECTION_BREAK;
            }
            pid = job_get_pid(job);
        } else {
            pid = (pid_t)parsed_number;
            job = job_from_pid(pid);

            if (!job_exists(job)) {
                sio_eprintf("(%s): No such process\n", tokens->argv[1]);
                CRITICAL_SECTION_BREAK;
            }
        }

        update_job(job, pid, state, &prev);
    }
}

static void bg(const struct cmdline_tokens *tokens) {
    continue_job(tokens, BG);
}

static void fg(const struct cmdline_tokens *tokens) {
    continue_job(tokens, FG);
}

struct stream_args {
    bool reset;
    bool append;
    bool enable;
    bool disable;
    char *args[2];
};

static void update_stream_context(struct stream_ctx *ctx,
                                  const struct stream_args *args,
                                  const struct argp *argp,
                                  const struct cmdline_tokens *tokens) {
    int flags = WR_FLAGS, fd;
    bool flags_present = args->append || args->reset;

    if (!flags_present && (args->enable || args->disable)) {
        // nothing else to do
        return;
    }

    if (args->append) {
        flags = APPEND_FLAGS;
    }

    MUTEX_CRITICAL_SECTION(&ctx->lock) {
        if (args->args[1] == NULL && !flags_present) {
            if (ctx->fd == STDOUT_FILENO) {
                puts("Output file: stdout");
            } else {
                sio_printf("Output file: %s\n", ctx->filepath);
            }
            puts(ctx->streaming ? "State: Enabled" : "State: Disabled");
            MUTEX_SECTION_BREAK;
        }

        if (args->reset) {
            if (ctx->fd != STDOUT_FILENO) {
                close(ctx->fd);
                free(ctx->filepath);
            }
            ctx->fd = STDOUT_FILENO;
            ctx->filepath = NULL;
            MUTEX_SECTION_BREAK;
        }

        if (args->args[1] == NULL) {
            argp_help(argp, stdout, ARGP_NO_EXIT, tokens->argv[0]);
            MUTEX_SECTION_BREAK;
        }

        if (ctx->fd != STDOUT_FILENO) {
            close(ctx->fd);
            free(ctx->filepath);
            ctx->filepath = NULL;
            ctx->fd = STDOUT_FILENO;
        }

        fd = open(args->args[1], flags, WR_PERMS);
        if (fd < 0) {
            perror("open");
            MUTEX_SECTION_BREAK;
        }

        ctx->filepath = malloc(strlen(args->args[1]) + 1);
        if (ctx->filepath == NULL) {
            perror("malloc");
            close(fd);
            MUTEX_SECTION_BREAK;
        }

        strcpy(ctx->filepath, args->args[1]);
        ctx->fd = fd;
    }
}

static bool update_stream_state(struct stream_ctx *ctx,
                                const struct stream_args *args) {
    if (!args->enable && !args->disable) {
        // no updates to the state
        return false;
    }

    MUTEX_CRITICAL_SECTION(&ctx->lock) {
        if (args->enable) {
            ctx->streaming = true;
            MUTEX_SECTION_BREAK;
        }

        ctx->streaming = false;
    }

    return true;
}

static const char config_stream_doc[] = "Manage the event streams.";
static const char config_stream_args_doc[] =
    "[error|exchange|neighbors|range] [FILE]...";
static struct argp_option config_stream_options[] = {
    {"reset", 'r', 0, 0, "Redirect stream events to stdout."},
    {"append", 'a', 0, 0, "Append the event reports to the given file."},
    {"disable", 'd', 0, 0, "Disable the given event stream."},
    {"enable", 'e', 0, 0,
     "Enable the given event stream. This takes precedence over --disable."},
    {0}};

static error_t parse_config_stream(int key, char *arg,
                                   struct argp_state *state) {
    struct stream_args *arguments = state->input;
    switch (key) {
    case 'r':
        arguments->reset = true;
        break;
    case 'a':
        arguments->append = true;
        break;
    case 'e':
        arguments->enable = true;
        break;
    case 'd':
        arguments->disable = true;
        break;
    case ARGP_KEY_ARG:
        if (state->arg_num < 2) {
            arguments->args[state->arg_num] = arg;
        }
        break;
    case ARGP_KEY_END:
        if (state->arg_num < 1) {
            argp_usage(state);
        }
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}

static void config_beluga_stream(const struct cmdline_tokens *tokens) {
    struct stream_args arguments = {};
    struct argp argp = {config_stream_options, parse_config_stream,
                        config_stream_args_doc, config_stream_doc};
    error_t err = argp_parse(&argp, tokens->argc, tokens->argv, ARGP_NO_EXIT, 0,
                             &arguments);
    struct stream_ctx *ctx;

    if (err != 0) {
        perror("argp_parse");
        return;
    }

    if (arguments.args[0] == NULL) {
        // help or usage invoked
        return;
    }

    if (strcmp(arguments.args[0], "error") == 0) {
        ctx = &attr.error;
    } else if (strcmp(arguments.args[0], "exchange") == 0) {
        ctx = &attr.exchange;
    } else if (strcmp(arguments.args[0], "neighbors") == 0) {
        ctx = &attr.neighbors;
    } else if (strcmp(arguments.args[0], "range") == 0) {
        ctx = &attr.ranges;
    } else {
        argp_help(&argp, stdout, ARGP_NO_EXIT, tokens->argv[0]);
        return;
    }

    update_stream_context(ctx, &arguments, &argp, tokens);
    update_stream_state(ctx, &arguments);
}

static void neighbors(const struct cmdline_tokens *tokens) {
    struct beluga_neighbor *list = NULL, *tmp;
    int len = 0;
    int fd = STDOUT_FILENO;

    if (tokens->outfile) {
        fd = open(tokens->outfile,
                  tokens->outfile_append ? APPEND_FLAGS : WR_FLAGS, WR_PERMS);
        if (fd < 0) {
            perror("open");
            return;
        }
    }

    MUTEX_CRITICAL_SECTION(&attr.serial_lock) {
        len = beluga_serial_get_neighbor_list(attr.serial, &list);
    }

    if (len < 0) {
        sio_dprintf(fd, "Insufficient memory\n");
        goto close_fd;
    }

    tmp = list;
    sio_dprintf(fd, "-----\n");
    sio_dprintf(fd, "Neighbor list:\n");
    for (int i = 0; i < len; i++) {
        sio_assert(tmp != NULL);
        sio_dprintf(fd, "ID: %d\n", tmp->id);
        sio_dprintf(fd, "RSSI: %d\n", tmp->rssi);
        sio_dprintf(fd, "Exchange: %d\n", tmp->exchange);
        sio_dprintf(fd, "Range: %f\n", tmp->range);
        sio_dprintf(fd, "Timestamp: %ld\n", tmp->time);

        if ((i + 1) < len) {
            sio_dprintf(fd, "\n");
        }
        tmp++;
    }
    sio_dprintf(fd, "-----\n");

    free(list);

close_fd:
    if (fd != STDOUT_FILENO) {
        close(fd);
    }
}
