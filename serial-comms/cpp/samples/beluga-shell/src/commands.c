/**
 * @file commands.c
 *
 * @brief
 *
 * @date 12/17/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <commands.h>
#include <errno.h>
#include <shell_helpers.h>
#include <stdio.h>

static void cmd_id(const struct cmdline_tokens *tokens);

static struct beluga_serial *_serial = NULL;
static struct command_info commands[] = {
    {"id", cmd_id},
};

int initialize_builtin_commands(struct beluga_serial *serial) {
    if (serial == NULL) {
        return -EINVAL;
    }
    _serial = serial;

    if (register_command_callbacks(commands, sizeof(commands) /
                                                 sizeof(commands[0])) < 0) {
        printf("Failed to initialize command handlers\n");
        exit(EXIT_FAILURE);
    }

    return 0;
}

static void cmd_id(const struct cmdline_tokens *tokens) {
    const char *arg = NULL;

    if (tokens->argc > 1) {
        arg = tokens->argv[1];
    }

    beluga_serial_id(_serial, arg);

    // todo: write to file if applicable
    printf("%s\n", _serial->response);
}
