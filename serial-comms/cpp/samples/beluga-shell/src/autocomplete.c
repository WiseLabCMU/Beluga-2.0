/**
 * @file autocomplete.c
 *
 * @brief
 *
 * @date 12/18/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <autocomplete.h>
#include <dirent.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

struct shell_command {
    const char *command;
    const char *path;
};

struct commands {
    struct shell_command *commands;
    size_t len;

    const char **path;
    size_t num_paths;
};

struct commands commands = {NULL, 0};

static bool check_executable(const char *path, const char *name) {
    char _path[FILENAME_MAX];
    struct stat sb;

    snprintf(_path, sizeof(_path), "%s/%s", path, name);
    return stat(_path, &sb) == 0 && ((sb.st_mode & S_IXUSR) != 0);
}

static bool in_list(const char *name) {
    for (size_t i = 0; i < commands.len; i++) {
        if (strcmp(name, commands.commands[i].command) == 0) {
            return true;
        }
    }
    return false;
}

static size_t count_files(const char *path, DIR *d) {
    size_t num_files = 0;
    struct dirent *entry;

    while ((entry = readdir(d)) != NULL) {
        if (entry->d_type != DT_DIR && check_executable(path, entry->d_name)) {
            num_files++;
        }
    }

    rewinddir(d);
    return num_files;
}

static int add_command(const char *name, const char *path) {
    commands.commands[commands.len].command = strdup(name);
    if (commands.commands[commands.len].command == NULL) {
        return -ENOMEM;
    }
    commands.commands[commands.len].path = path;
    commands.len++;
    return 0;
}

static int add_commands_to_list(const char *path, DIR *d) {
    struct dirent *entry;
    const char *_path = strdup(path);
    commands.path[commands.num_paths] = _path;
    commands.num_paths++;

    if (_path == NULL) {
        return -ENOMEM;
    }

    while ((entry = readdir(d)) != NULL) {
        if (entry->d_type != DT_DIR && check_executable(path, entry->d_name) &&
            !in_list(entry->d_name)) {
            if (add_command(entry->d_name, _path) < 0) {
                return -ENOMEM;
            }
        }
    }

    return 0;
}

static int add_path_directory(const char *path) {
    DIR *d;
    size_t num_commands;
    int ret;

    d = opendir(path);

    if (d == NULL) {
        return -EBADF;
    }

    num_commands = count_files(path, d);

    if (num_commands <= 0) {
        return 0;
    }

    commands.commands =
        reallocarray(commands.commands, commands.len + num_commands,
                     sizeof(struct shell_command));

    if (errno == ENOMEM) {
        return -errno;
    }

    ret = add_commands_to_list(path, d);

    closedir(d);

    return ret;
}

static size_t count_paths(const char *path) {
    const char delim = ':';
    size_t paths = 1;
    char *buf = (char *)path;
    char *endbuf = buf + strlen(buf);

    if (*path == '\0') {
        return 0;
    }

    while (buf < endbuf) {
        if (*buf == delim && *(buf + 1) != '\0') {
            paths++;
        }
        buf++;
    }

    return paths;
}

static int qsort_comp(const void *left, const void *right) {
    const struct shell_command *l_cmd = left;
    const struct shell_command *r_cmd = right;

    return strcmp(l_cmd->command, r_cmd->command);
}

static int build_commands_list(void) {
    const char *path = getenv("PATH");
    char *path_copy, *token, *pos;
    int err;

    if (path == NULL) {
        printf("PATH does not exists\n");
        return -EACCES;
    }

    path_copy = strdup(path);
    if (path_copy == NULL) {
        return -ENOMEM;
    }

    commands.path =
        (const char **)calloc(count_paths(path), sizeof(const char **));

    token = strtok_r(path_copy, ":", &pos);
    while (token != NULL) {
        if ((err = add_path_directory(token)) < 0) {
            return err;
        }
        token = strtok_r(NULL, ":", &pos);
    }
    free(path_copy);

    commands.path =
        reallocarray(commands.path, commands.num_paths, sizeof(const char **));

    qsort(commands.commands, commands.len, sizeof(struct shell_command),
          qsort_comp);

    return 0;
}

int initialize_autocomplete(void) { return build_commands_list(); }

void cleanup_autocomplete(void) {
    for (size_t i = 0; i < commands.len; i++) {
        free((void *)commands.commands[i].command);
    }
    free(commands.commands);
    commands.commands = NULL;
    commands.len = 0;

    for (size_t i = 0; i < commands.num_paths; i++) {
        free((void *)commands.path[i]);
    }
    free(commands.path);
    commands.path = NULL;
    commands.num_paths = 0;
}

static struct shell_command *find_command(const char *command) {
    ssize_t low = 0;
    ssize_t high = (ssize_t)commands.len - 1;

    while (low <= high) {
        ssize_t mid = low + ((high - low) / 2);
        int res = strcmp(commands.commands[mid].command, command);

        if (res == 0) {
            return &commands.commands[mid];
        }

        if (res < 0) {
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }

    return NULL;
}

int autocomplete_register_builtin_command(const char *command) {
    struct shell_command *cmd = find_command(command);

    if (cmd != NULL) {
        cmd->path = NULL;
        return 0;
    }

    commands.commands = reallocarray(commands.commands, commands.len + 1,
                                     sizeof(struct shell_command));
    if (errno == ENOMEM) {
        return -ENOMEM;
    }

    add_command(command, NULL);
    qsort(commands.commands, commands.len, sizeof(struct shell_command),
          qsort_comp);

    return 0;
}

const char *command_path(const char *command) {
    struct shell_command *cmd = find_command(command);

    if (cmd == NULL) {
        return NULL;
    }

    return cmd->path;
}
