/**
 * @file shell_helpers.h
 *
 * @brief
 *
 * @date 12/15/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_SHELL_HELPERS_H
#define BELUGA_SHELL_HELPERS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

extern char **environ;

#define MAXJOBS 64

#if MAXJOBS >= UINT32_MAX
typedef uint64_t jid_t;
#elif MAXJOBS >= UINT16_MAX
typedef uint32_t jid_t;
#elif MAXJOBS >= UINT8_MAX
typedef uint16_t jid_t;
#else
typedef uint8_t jid_t;
#endif

#define ARG_UNUSED(arg) (void)(arg)

enum job_state {
    FG,
    BG,
    ST,
    UNDEF,
};

enum parseline_result {
    PARSELINE_FG,
    PARSELINE_BG,
    PARSELINE_EMPTY,
    PARSELINE_ERROR,
};

struct cmdline_tokens {
    int argc;
    char **argv;
    char *infile;
    char *outfile;
    bool builtin_command;
    size_t builtin_index;
    char *_buf;
};

typedef void (*command_callback_t)(int argc, char **argv);
struct command_info {
    const char *cmd_str;
    command_callback_t callback;
};

int register_command_callbacks(const struct command_info *commands,
                               size_t num_handlers);

enum parseline_result parseline(const char *cmdline,
                                struct cmdline_tokens *tokens);
void cleanup_parseline(struct cmdline_tokens *tokens);

void run_builtin_command(struct cmdline_tokens *tokens);

void init_job_list(void);
void destroy_job_list(void);
jid_t add_job(pid_t pid, enum job_state state, const char *cmdline);
bool delete_job(jid_t jid);
jid_t fg_job(void);
bool job_exists(jid_t jid);
jid_t job_from_pid(pid_t pid);
pid_t job_get_pid(jid_t jid);
const char *job_get_cmdline(jid_t jid);
enum job_state job_get_state(jid_t jid);
void job_set_state(jid_t jid, enum job_state state);
bool list_jobs(int output_fd);

typedef void signal_handler_t(int);
signal_handler_t *Signal(int signum, signal_handler_t *handler);

#endif // BELUGA_SHELL_HELPERS_H