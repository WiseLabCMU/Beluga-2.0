/**
 * @file main.c
 *
 * @brief
 *
 * @date 12/12/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <autocomplete.h>
#include <beluga_serial_c_api.h>
#include <commands.h>
#include <critical_sections.h>
#include <errno.h>
#include <fcntl.h>
#include <readline/history.h>
#include <readline/readline.h>
#include <select_port.h>
#include <shell_helpers.h>
#include <signal.h>
#include <sio.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#define MAX_HISTORY 500

static const char *history_file = ".history";
static const char *beluga_shell_cache_dir = ".beluga-shell";
static struct beluga_serial *serial = NULL;
static char *port = NULL;

static const char *search_paths[] = {"/bin/", "/usr/bin/", "/usr/games/"};

sig_atomic_t fg_running = 0;

static void cleanup(void);
static void init(void);
static void run(void);
static bool evaluate_command(const char *cmd);
static void sigchld_handler(int sig);
static void sigint_handler(int sig);
static void sigtstp_handler(int sig);

int main(int argc, char *argv[]) {
    // todo

    init();

    run();

    return 0;
}

static void cache_mkdir(void) {
    char path[FILENAME_MAX];
    int err;

    int bytes_written = snprintf(path, FILENAME_MAX, "%s/%s", getenv("HOME"),
                                 beluga_shell_cache_dir);
    if (bytes_written == 0 || bytes_written >= FILENAME_MAX) {
        printf("Unable to save history: path too long");
        exit(EXIT_FAILURE);
    }

    err = mkdir(path, 0700);

    if (err != 0 && errno != EEXIST) {
        perror("mkdir()");
        exit(EXIT_FAILURE);
    }
}

static void load_history(void) {
    char path[FILENAME_MAX];
    int err,
        bytes_written = snprintf(path, FILENAME_MAX, "%s/%s/%s", getenv("HOME"),
                                 beluga_shell_cache_dir, history_file);
    if (bytes_written == 0 || bytes_written >= FILENAME_MAX) {
        printf("Unable to save history: path too long");
        exit(EXIT_FAILURE);
    }

    err = read_history(path);

    if (err != 0 && err != ENOENT) {
        perror("Failed to read history");
        exit(EXIT_FAILURE);
    }
}

static void init(void) {
    struct beluga_serial_attr attr = {
        .baud = BAUD_115200,
        .timeout = 2000,
        .serial_timeout = 100,
        .reboot_event = report_unexpected_reboot,
        .fatal_error_event = report_fatal_error,
    };

    cache_mkdir();

    if (atexit(cleanup) < 0) {
        perror("atexit()");
        exit(EXIT_FAILURE);
    }

    stifle_history(MAX_HISTORY);

    port = pick_port();
    attr.port = port;

    load_history();

    if (setvbuf(stdout, NULL, _IOLBF, 0) < 0) {
        perror("setvbuf error");
        exit(1);
    }

    if (initialize_autocomplete() < 0) {
        exit(EXIT_FAILURE);
    }

    serial = create_beluga_serial_instance(&attr);

    if (serial == NULL) {
        perror("create_beluga_serial_instance()");
        exit(EXIT_FAILURE);
    }

    beluga_serial_start(serial);
    printf("Connection established\n\n");

    init_job_list();
    initialize_builtin_commands(serial);

    Signal(SIGINT, sigint_handler);
    Signal(SIGTSTP, sigtstp_handler);
    Signal(SIGCHLD, sigchld_handler);

    Signal(SIGTTIN, SIG_IGN);
    Signal(SIGTTOU, SIG_IGN);
}

static void run(void) {
    char *command;
    bool add_hist = true;

    while (true) {
        command = readline("beluga-shell> ");

        if (command == NULL) {
            if (errno == 0) {
                exit(EXIT_SUCCESS);
            }
            perror("readline()");
            exit(EXIT_FAILURE);
        }

        add_hist = evaluate_command(command);

        if (add_hist) {
            add_history(command);
        }

        free(command);
    }
}

static void redirect_io(struct cmdline_tokens *tokens) {
    int input_fd = STDIN_FILENO, output_fd = STDOUT_FILENO;

    if (tokens->infile != NULL) {
        input_fd = open(tokens->infile, O_RDONLY);

        if (input_fd < 0) {
            perror(tokens->infile);
            _exit(EXIT_FAILURE);
        }

        if (dup2(input_fd, STDIN_FILENO) < 0) {
            perror("dup2()");
            _exit(EXIT_FAILURE);
        }
    }

    if (tokens->outfile) {

        if (!tokens->outfile_append) {
            output_fd = open(tokens->outfile, WR_FLAGS, WR_PERMS);
        } else {
            output_fd = open(tokens->outfile, APPEND_FLAGS, WR_PERMS);
        }

        if (output_fd < 0) {
            perror(tokens->outfile);
            _exit(EXIT_FAILURE);
        }

        if (dup2(output_fd, STDOUT_FILENO) < 0) {
            perror("dup2()");
            _exit(EXIT_FAILURE);
        }
    }
}

static void run_shell_command(struct cmdline_tokens *tokens) {
    char line[FILENAME_MAX];
    const char *path = command_path(tokens->argv[0]);

    snprintf(line, sizeof(line), "%s/%s", path, tokens->argv[0]);
    execve(line, tokens->argv, environ);

    sio_eprintf("%s: %s\n", tokens->argv[0], strerror(errno));
    _exit(EXIT_FAILURE);
}

static void run_job(struct cmdline_tokens *tokens, const char *cmdline,
                    enum job_state state) {
    sigset_t mask, prev;
    pid_t pid;

    CRITICAL_SECTION(mask, prev) {
        fg_running = state == FG;
        if ((pid = fork()) == 0) {
            setpgrp();
            redirect_io(tokens);
            CRITICAL_SECTION_EXIT_FUNCTION(prev, run_shell_command(tokens), );
        }

        add_job(pid, state, cmdline);

        if (state == FG) {
            while (fg_running) {
                sigsuspend(&prev);
            }
        } else {
            sio_printf("[%d] (%ld) %s\n", (int)job_from_pid(pid), (long)pid,
                       cmdline);
        }
    }
}

static bool evaluate_command(const char *cmd) {
    struct cmdline_tokens tokens;
    enum parseline_result result = parseline(cmd, &tokens);

    if (tokens.builtin_command) {
        run_builtin_command(&tokens);
    } else if (result != PARSELINE_ERROR && result != PARSELINE_EMPTY) {
        run_job(&tokens, cmd, result == PARSELINE_FG ? FG : BG);
    }

    cleanup_parseline(&tokens);

    return result != PARSELINE_ERROR && result != PARSELINE_EMPTY;
}

static void save_history(void) {
    char path[FILENAME_MAX];

    int bytes_written = snprintf(path, FILENAME_MAX, "%s/%s/%s", getenv("HOME"),
                                 beluga_shell_cache_dir, history_file);
    if (bytes_written == 0 || bytes_written >= FILENAME_MAX) {
        printf("Unable to save history: path too long");
        return;
    }

    write_history(path);
}

static void sigchld_handler(int sig) {
    ARG_UNUSED(sig);
    pid_t pid;
    jid_t job;
    int status;

    SIGNAL_HANDLER_CRIT_SECTION() {
        while ((pid = waitpid((pid_t)-1, &status, WNOHANG | WUNTRACED)) > 0) {
            job = job_from_pid(pid);
            if (job == fg_job()) {
                fg_running = 0;
            }

            if (WIFEXITED(status) || WIFSIGNALED(status)) {
                if (WIFSIGNALED(status)) {
                    sio_printf("Job [%d] (%ld) terminated by signal %d\n",
                               (int)job, (long)pid, WTERMSIG(status));
                }
                delete_job(job);
            } else if (WIFSTOPPED(status)) {
                sio_printf("Job [%d] (%ld) stopped by signal %d\n", (int)job,
                           (long)pid, WSTOPSIG(status));
                job_set_state(job, ST);
            } else {
                delete_job(job);
            }
        }
    }
}

static void sigint_handler(int sig) {
    ARG_UNUSED(sig);
    pid_t pid;
    jid_t job;

    SIGNAL_HANDLER_CRIT_SECTION() {
        if ((job = fg_job()) != 0) {
            pid = job_get_pid(job);

            kill(-pid, SIGINT);
        }
    }
}

static void sigtstp_handler(int sig) {
    ARG_UNUSED(sig);
    jid_t job;

    SIGNAL_HANDLER_CRIT_SECTION() {
        job = fg_job();

        if (job != 0) {
            pid_t pid = job_get_pid(job);
            kill(-pid, SIGTSTP);
        }
    }
}

static void cleanup(void) {
    Signal(SIGINT, SIG_DFL);
    Signal(SIGCHLD, SIG_DFL);
    Signal(SIGTSTP, SIG_DFL);
    Signal(SIGTTIN, SIG_DFL);
    Signal(SIGTTOU, SIG_DFL);
    Signal(SIGQUIT, SIG_DFL);

    save_history();

    destroy_beluga_serial_instance(&serial);
    free(port);

    cleanup_autocomplete();
}
