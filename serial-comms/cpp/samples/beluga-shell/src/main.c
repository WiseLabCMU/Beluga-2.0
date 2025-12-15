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

static struct beluga_serial *serial;
static char *port;

static void cleanup(void);
static void init(void);
static void run(void);

int main(int argc, char *argv[]) {
    // todo

    init();

    run();

    return 0;
}

static void init(void) {
    struct beluga_serial_attr attr = {
        .baud = BAUD_115200, .timeout = 2000, .serial_timeout = 100};

    if (atexit(cleanup) < 0) {
        perror("atexit()");
        exit(EXIT_FAILURE);
    }

    // todo: create autocompletion

    port = pick_port();
    attr.port = port;

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
    bool add_hist = false;

    while (true) {
        command = readline("beluga-shell> ");

        if (command == NULL) {
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

static void cleanup(void) {
    Signal(SIGINT, SIG_DFL);
    Signal(SIGCHLD, SIG_DFL);
    Signal(SIGTSTP, SIG_DFL);
    Signal(SIGTTIN, SIG_DFL);
    Signal(SIGTTOU, SIG_DFL);
    Signal(SIGQUIT, SIG_DFL);

    destroy_beluga_serial_instance(&serial);
    free(port);
}
