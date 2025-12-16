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

struct job {
    pid_t pid;
    jid_t jid;
    enum job_state state;
    char *cmdline;
};

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

static struct job job_list[MAXJOBS];
static jid_t nextjid = 1;
static bool jobs_initialized;

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

static void check_blocked(void) {
    if (!jobs_initialized) {
        // todo
    }

    sigset_t currmask;
    sigprocmask(SIG_SETMASK, NULL, &currmask);

    bool sigchld = sigismember(&currmask, SIGCHLD) <= 0;
    bool sigint = sigismember(&currmask, SIGINT) <= 0;
    bool sigtstp = sigismember(&currmask, SIGTSTP) <= 0;

    if (sigchld || sigint || sigtstp) {
        // todo
    }
}

static struct job *get_job(jid_t jid) {
    if (jid < 1 || jid > MAXJOBS) {
        // todo
        abort();
    }
    return &job_list[jid - 1];
}

static void clearjob(struct job *job) {
    job->pid = 0;
    job->jid = 0;
    job->state = UNDEF;
}

void init_job_list(void) {
    jobs_initialized = true;
    for (jid_t jid = 1; jid <= MAXJOBS; jid++) {
        struct job *job = get_job(jid);
        clearjob(job);
        job->cmdline = NULL;
    }
    nextjid = 1;
}

void destroy_job_list(void) {
    for (jid_t jid = 1; jid <= MAXJOBS; jid++) {
        struct job *job = get_job(jid);
        clearjob(job);
        free(job->cmdline);
        job->cmdline = NULL;
    }
    nextjid = 1;
}

static jid_t maxjid(void) {
    for (jid_t jid = MAXJOBS; jid > 0; jid--) {
        if (job_exists(jid)) {
            return jid;
        }
    }
    return 0;
}

bool job_exists(jid_t jid) {
    check_blocked();

    if (jid < 1 || jid > MAXJOBS) {
        return false;
    }

    struct job *job = get_job(jid);
    return job->state != UNDEF;
}

jid_t add_job(pid_t pid, enum job_state state, const char *cmdline) {
    check_blocked();
    if (!((state == FG && fg_job() == 0) || state == BG || state == ST)) {
        if (state == FG) {
            // todo
            abort();
        }
        // todo
        abort();
    }

    if (pid <= 0) {
        // todo
        abort();
    }

    if (cmdline == NULL) {
        // todo
        abort();
    }

    // todo
    if (nextjid > MAXJOBS) {
        return 0;
    }

    struct job *job = get_job(nextjid);
    // todo

    job->jid = nextjid;
    job->pid = pid;
    job->state = state;

    job->cmdline = realloc(job->cmdline, strlen(cmdline) - 1);
    if (job->cmdline == NULL) {
        // todo
        _exit(EXIT_FAILURE);
    }

    strcpy(job->cmdline, cmdline);

    nextjid++;
    // todo
    return job->jid;
}

bool delete_job(jid_t jid) {
    check_blocked();

    if (!job_exists(jid)) {
        return false;
    }

    struct job *job = get_job(jid);
    clearjob(job);

    nextjid = maxjid() + 1;
    // todo
    return true;
}

jid_t fg_job(void) {
    check_blocked();

    for (jid_t jid = 1; jid <= MAXJOBS; jid++) {
        struct job *job = get_job(jid);
        if (job->state == FG) {
            return jid;
        }
    }

    return 0;
}

jid_t job_from_pid(pid_t pid) {
    check_blocked();

    if (pid < 1) {
        return 0;
    }

    for (jid_t jid = 1; jid <= MAXJOBS; jid++) {
        struct job *job = get_job(jid);
        if (job->state != UNDEF && job->pid == pid) {
            return jid;
        }
    }

    return 0;
}

pid_t job_get_pid(jid_t jid) {
    check_blocked();
    // todo

    struct job *job = get_job(jid);
    return job->pid;
}

const char *job_get_cmdline(jid_t jid) {
    check_blocked();
    // todo

    struct job *job = get_job(jid);
    return job->cmdline;
}

enum job_state job_get_state(jid_t jid) {
    check_blocked();
    // todo

    struct job *job = get_job(jid);
    return job->state;
}

void job_set_state(jid_t jid, enum job_state state) {
    check_blocked();
    // todo
    // todo

    struct job *job = get_job(jid);
    job->state = state;
}

bool list_jobs(int output_fd) {
    check_blocked();
    if (output_fd < 0) {
        // todo
        abort();
    }

    for (jid_t jid = 1; jid <= MAXJOBS; jid++) {
        struct job *job = get_job(jid);
        if (job->state == UNDEF) {
            continue;
        }

        char *status = NULL;
        switch (job->state) {
        case BG: {
            status = "Running    ";
            break;
        }
        case FG: {
            status = "Foreground ";
            break;
        }
        case ST: {
            status = "Stopped    ";
            break;
        }
        default: {
            // todo
            abort();
        }
        }

        // todo
        if (false) {
            // todo
            return false;
        }
    }

    return true;
}
