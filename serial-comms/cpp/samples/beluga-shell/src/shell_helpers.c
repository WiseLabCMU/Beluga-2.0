/**
 * @file shell_helpers.c
 *
 * @brief
 *
 * @date 12/15/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <assert.h>
#include <errno.h>
#include <shell_helpers.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct builtin_commands {
    const struct command_info *commands;
    size_t len;
};

enum parse_state {
    PARSE_STATE_NORMAL,
    PARSE_STATE_INFILE,
    PARSE_STATE_OUTFILE,
};

struct builtin_commands builtin_commands = {
    NULL,
    0,
};

signal_handler_t *Signal(int signum, signal_handler_t handler) {
    struct sigaction action, old_action;

    action.sa_handler = handler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_RESTART;

    if (sigaction(signum, &action, &old_action) < 0) {
        perror("Signal error");
        exit(EXIT_FAILURE);
    }

    return old_action.sa_handler;
}

int register_command_callbacks(const struct command_info *commands,
                               size_t num_handlers) {
    if (commands == NULL || num_handlers == 0) {
        return -EINVAL;
    }

    builtin_commands.commands = commands;
    builtin_commands.len = num_handlers;
    return 0;
}

static int validate_and_count_args(struct cmdline_tokens *tokens) {
    const char *delims = " \t\r\n";
    char *buf;
    char *next;
    char *endbuf;
    enum parse_state state = PARSE_STATE_NORMAL;

    buf = tokens->_buf;
    endbuf = buf + strlen(buf);

    while (buf < endbuf) {
        buf += strspn(buf, delims);
        if (buf >= endbuf) {
            break;
        }

        switch (*buf) {
        case '<': {
            if (tokens->infile != NULL) {
                return -EEXIST;
            }
            state = PARSE_STATE_INFILE;
            buf++;
            continue;
        }
        case '>': {
            if (tokens->outfile != NULL) {
                return -EEXIST;
            }
            state = PARSE_STATE_OUTFILE;
            buf++;
            continue;
        }
        case '\'':
        case '\"': {
            buf++;
            next = strchr(buf, *(buf - 1));
            break;
        }
        default: {
            next = buf + strcspn(buf, delims);
            break;
        }
        }

        if (next == NULL) {
            return -EINVAL;
        }

        switch (state) {
        case PARSE_STATE_NORMAL: {
            tokens->argc += 1;
            break;
        }
        case PARSE_STATE_INFILE: {
            tokens->infile = buf;
            break;
        }
        case PARSE_STATE_OUTFILE: {
            tokens->outfile = buf;
            break;
        }
        default: {
            return -EBADMSG;
        }
        }

        state = PARSE_STATE_NORMAL;
        buf = next + 1;
    }

    return 0;
}

int parse_arguments(struct cmdline_tokens *tokens) {
    const char *delims = " \t\r\n";
    char *buf;
    char *next;
    char *endbuf;
    int argc = 0;
    enum parse_state state = PARSE_STATE_NORMAL;

    buf = tokens->_buf;
    endbuf = buf + strlen(buf);

    while (buf < endbuf) {
        buf += strspn(buf, delims);
        if (buf >= endbuf) {
            break;
        }

        switch (*buf) {
        case '<': {
            state = PARSE_STATE_INFILE;
            buf++;
            continue;
        }
        case '>': {
            state = PARSE_STATE_OUTFILE;
            buf++;
            continue;
        }
        case '\'':
        case '\"': {
            buf++;
            next = strchr(buf, *(buf - 1));
            break;
        }
        default: {
            next = buf + strcspn(buf, delims);
            break;
        }
        }

        if (next == NULL) {
            return -EINVAL;
        }

        *next = '\0';

        switch (state) {
        case PARSE_STATE_NORMAL: {
            tokens->argv[argc] = buf;
            argc += 1;
            assert(argc <= tokens->argc);
            break;
        }
        case PARSE_STATE_INFILE: {
            tokens->infile = buf;
            break;
        }
        case PARSE_STATE_OUTFILE: {
            tokens->outfile = buf;
            break;
        }
        default: {
            return -EBADMSG;
        }
        }

        state = PARSE_STATE_NORMAL;
        buf = next + 1;
    }

    return 0;
}

static void check_if_builtin(struct cmdline_tokens *tokens) {
    for (size_t i = 0; i < builtin_commands.len; i++) {
        if (strcmp(tokens->argv[0], builtin_commands.commands[i].cmd_str) ==
            0) {
            tokens->builtin_command = true;
            tokens->builtin_index = i;
            return;
        }
    }
}

enum parseline_result parseline(const char *cmdline,
                                struct cmdline_tokens *tokens) {
    assert(tokens != NULL);

    if (cmdline == NULL || *cmdline == '\0') {
        return PARSELINE_EMPTY;
    }

    tokens->_buf = calloc(strlen(cmdline) + 1, sizeof(char));

    if (tokens->_buf == NULL) {
        return PARSELINE_ERROR;
    }

    memcpy(tokens->_buf, cmdline, strlen(cmdline));

    tokens->argc = 0;
    tokens->infile = NULL;
    tokens->outfile = NULL;
    tokens->builtin_command = false;

    if (validate_and_count_args(tokens) < 0) {
        return PARSELINE_ERROR;
    }

    if (tokens->argc == 0) {
        return PARSELINE_EMPTY;
    }

    tokens->argv = (char **)calloc(tokens->argc + 1, sizeof(char *));

    if (tokens->argv == NULL) {
        free(tokens->_buf);
        return PARSELINE_ERROR;
    }

    if (parse_arguments(tokens) < 0) {
        return PARSELINE_ERROR;
    }

    check_if_builtin(tokens);

    if (*tokens->argv[tokens->argc - 1] == '&') {
        tokens->argc--;
        tokens->argv[tokens->argc] = NULL;
        if (tokens->argc == 0) {
            return PARSELINE_EMPTY;
        }
        return PARSELINE_BG;
    }
    return PARSELINE_FG;
}

void cleanup_parseline(struct cmdline_tokens *tokens) {
    assert(tokens != NULL);

    if (tokens->argv != NULL) {
        free(tokens->argv);
        tokens->argv = NULL;
    }

    if (tokens->_buf != NULL) {
        free(tokens->_buf);
        tokens->_buf = NULL;
    }
}

void run_builtin_command(struct cmdline_tokens *tokens) {
    assert(tokens->builtin_index < builtin_commands.len);

    builtin_commands.commands[tokens->builtin_index].callback(tokens->argc,
                                                              tokens->argv);
}
