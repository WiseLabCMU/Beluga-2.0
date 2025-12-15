/**
 * @file main.c
 *
 * @brief
 *
 * @date 12/12/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <beluga_serial_c_api.h>
#include <errno.h>
#include <readline/history.h>
#include <readline/readline.h>
#include <select_port.h>
#include <shell_helpers.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

#define MAX_HISTORY 500

static const char *history_file = ".history";
static const char *beluga_shell_cache_dir = ".beluga-shell";
static struct beluga_serial *serial = NULL;
static char *port = NULL;
static bool history_initialized = false;

static void cleanup(void);
static void init(void);
static void run(void);

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
        .baud = BAUD_115200, .timeout = 2000, .serial_timeout = 100};

    cache_mkdir();

    if (atexit(cleanup) < 0) {
        perror("atexit()");
        exit(EXIT_FAILURE);
    }

    stifle_history(MAX_HISTORY);

    port = pick_port();
    attr.port = port;

    load_history();

    // todo: create autocompletion
    serial = create_beluga_serial_instance(&attr);

    if (serial == NULL) {
        perror("create_beluga_serial_instance()");
        exit(EXIT_FAILURE);
    }

    beluga_serial_start(serial);
    printf("Connection established\n\n");
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

        // todo: run command

        if (add_hist) {
            add_history(command);
        }

        free(command);
    }
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
}
