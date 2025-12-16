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
#include <sio.h>
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
        sio_eprintf("WARNING: Failed to call init_job_list()\n");
    }

    sigset_t currmask;
    sigprocmask(SIG_SETMASK, NULL, &currmask);

    bool sigchld = sigismember(&currmask, SIGCHLD) <= 0;
    bool sigint = sigismember(&currmask, SIGINT) <= 0;
    bool sigtstp = sigismember(&currmask, SIGTSTP) <= 0;

    if (sigchld || sigint || sigtstp) {
        sio_eprintf("WARNING: signals not blocked before accessing job list:"
                    "%s%s%s\n",
                    sigchld ? " SIGCHLD" : "", sigint ? " SIGINT" : "",
                    sigtstp ? " SIGTSTP" : "");
    }
}

static struct job *get_job(jid_t jid) {
    if (jid < 1 || jid > MAXJOBS) {
        sio_eprintf("get_job(): invalid jid\n");
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
            sio_eprintf("add_job: foreground job already exists\n");
            abort();
        }
        sio_eprintf("add_job: invalid job state\n");
        abort();
    }

    if (pid <= 0) {
        sio_eprintf("add_job: invalid pid\n");
        abort();
    }

    if (cmdline == NULL) {
        sio_eprintf("add_job: missing command line\n");
        abort();
    }

    sio_assert(nextjid > 0);
    if (nextjid > MAXJOBS) {
        return 0;
    }

    struct job *job = get_job(nextjid);
    sio_assert(job->state == UNDEF);

    job->jid = nextjid;
    job->pid = pid;
    job->state = state;

    job->cmdline = realloc(job->cmdline, strlen(cmdline) - 1);
    if (job->cmdline == NULL) {
        sio_eprintf("Realloc error\n");
        _exit(EXIT_FAILURE);
    }

    strcpy(job->cmdline, cmdline);

    nextjid++;
    sio_assert(nextjid > 0);
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
    sio_assert(nextjid > 0);
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

static void require_job_exists(char *func, jid_t jid) {
    if (!job_exists(jid)) {
        sio_eprintf("FATAL: %s: invalid JID argument %d\n", func, jid);
        abort();
    }
}

static void require_valid_state(jid_t jid, enum job_state state) {
    if (state != FG && state != BG && state != ST) {
        sio_eprintf("FATAL: job_set_state: invalid job state: %d\n", state);
        abort();
    }
    if (state == FG && fg_job() != 0) {
        sio_eprintf("FATAL: job_set_state: cannot make %d the foreground job "
                    "when another job %d is also the foreground job",
                    jid, fg_job());
        abort();
    }
}

pid_t job_get_pid(jid_t jid) {
    check_blocked();
    require_job_exists("job_get_pid()", jid);

    struct job *job = get_job(jid);
    return job->pid;
}

const char *job_get_cmdline(jid_t jid) {
    check_blocked();
    require_job_exists("job_get_cmdline()", jid);

    struct job *job = get_job(jid);
    return job->cmdline;
}

enum job_state job_get_state(jid_t jid) {
    check_blocked();
    require_job_exists("job_set_state", jid);

    struct job *job = get_job(jid);
    return job->state;
}

void job_set_state(jid_t jid, enum job_state state) {
    check_blocked();
    require_job_exists("job_set_state", jid);
    require_valid_state(jid, state);

    struct job *job = get_job(jid);
    job->state = state;
}

bool list_jobs(int output_fd) {
    check_blocked();
    if (output_fd < 0) {
        sio_eprintf("list_jobs(): invalid file descriptor\n");
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
            sio_eprintf("Invalid job state\n");
            abort();
        }
        }

        ssize_t res = sio_dprintf(output_fd, "[%d] (%d) %s%s\n", job->jid,
                                  job->pid, status, job->cmdline);
        if (res < 0) {
            sio_eprintf("list_jobs: Error writing to output_fd: %d\n",
                        output_fd);
            return false;
        }
    }

    return true;
}
