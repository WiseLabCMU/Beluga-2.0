/**
 * @file commands.c
 *
 * @brief
 *
 * @date 12/17/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <autocomplete.h>
#include <commands.h>
#include <critical_sections.h>
#include <errno.h>
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
static void jobs(const struct cmdline_tokens *tokens);
static void bg(const struct cmdline_tokens *tokens);
static void fg(const struct cmdline_tokens *tokens);
static void stream_exchanges(const struct cmdline_tokens *tokens);
static void stream_ranges(const struct cmdline_tokens *tokens);
static void stream_neighbors(const struct cmdline_tokens *tokens);

static struct beluga_serial *_serial = NULL;
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
    COMMAND(jobs),
    COMMAND(bg),
    COMMAND(fg),
};

#define ARRAY_SIZE(array_) sizeof(array_) / sizeof(array_[0])

int initialize_builtin_commands(struct beluga_serial *serial) {
    if (serial == NULL) {
        return -EINVAL;
    }
    _serial = serial;

    if (register_command_callbacks(commands, ARRAY_SIZE(commands)) < 0) {
        printf("Failed to initialize command handlers\n");
        exit(EXIT_FAILURE);
    }

    for (size_t i = 0; i < ARRAY_SIZE(commands); i++) {
        autocomplete_register_builtin_command(commands[i].cmd_str);
    }

    return 0;
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

    sio_dprintf(output_fd, "%s\n", _serial->response);
}

static void id(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_id(_serial, arg);
    write_serial_response(tokens);
}

static void start_ble(const struct cmdline_tokens *tokens) {
    beluga_serial_start_ble(_serial);
    write_serial_response(tokens);
}

static void stop_ble(const struct cmdline_tokens *tokens) {
    beluga_serial_stop_ble(_serial);
    write_serial_response(tokens);
}

static void start_uwb(const struct cmdline_tokens *tokens) {
    beluga_serial_start_uwb(_serial);
    write_serial_response(tokens);
}

static void stop_uwb(const struct cmdline_tokens *tokens) {
    beluga_serial_stop_uwb(_serial);
    write_serial_response(tokens);
}

static void bootmode(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_bootmode(_serial, arg);
    write_serial_response(tokens);
}

static void rate(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_rate(_serial, arg);
    write_serial_response(tokens);
}

static void channel(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_channel(_serial, arg);
    write_serial_response(tokens);
}

static void reset(const struct cmdline_tokens *tokens) {
    beluga_serial_reset(_serial);
    write_serial_response(tokens);
}

static void timeout(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_timeout(_serial, arg);
    write_serial_response(tokens);
}

static void txpower(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    // todo
    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_txpower(_serial, arg);
    write_serial_response(tokens);
}

static void streammode(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_streammode(_serial, arg);
    write_serial_response(tokens);
}

static void twrmode(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_twrmode(_serial, arg);
    write_serial_response(tokens);
}

static void ledmode(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_ledmode(_serial, arg);
    write_serial_response(tokens);
}

static void reboot(const struct cmdline_tokens *tokens) {
    beluga_serial_reboot(_serial);
    write_serial_response(tokens);
}

static void pwramp(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_pwramp(_serial, arg);
    write_serial_response(tokens);
}

static void antenna(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_antenna(_serial, arg);
    write_serial_response(tokens);
}

static void cmd_time(const struct cmdline_tokens *tokens) {
    beluga_serial_time(_serial);
    write_serial_response(tokens);
}

static void deepsleep(const struct cmdline_tokens *tokens) {
    beluga_serial_deepsleep(_serial);
    write_serial_response(tokens);
}

static void datarate(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_datarate(_serial, arg);
    write_serial_response(tokens);
}

static void preamble(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_preamble(_serial, arg);
    write_serial_response(tokens);
}

static void pulserate(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_pulserate(_serial, arg);
    write_serial_response(tokens);
}

static void phr(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_phr(_serial, arg);
    write_serial_response(tokens);
}

static void pac(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_pac(_serial, arg);
    write_serial_response(tokens);
}

static void sfd(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_sfd(_serial, arg);
    write_serial_response(tokens);
}

static void panid(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_panid(_serial, arg);
    write_serial_response(tokens);
}

static void evict(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_evict(_serial, arg);
    write_serial_response(tokens);
}

static void verbose(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_verbose(_serial, arg);
    write_serial_response(tokens);
}

static void status(const struct cmdline_tokens *tokens) {
    beluga_serial_status(_serial);
    write_serial_response(tokens);
}

static void version(const struct cmdline_tokens *tokens) {
    beluga_serial_version(_serial);
    write_serial_response(tokens);
}

static void starve(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_starve(_serial, arg);
    write_serial_response(tokens);
}

static void exchange(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_exchange(_serial, arg);
    write_serial_response(tokens);
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
