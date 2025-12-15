/**
 * @file select_port.c
 *
 * @brief
 *
 * @date 12/15/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <beluga_serial_c_api.h>
#include <readline/readline.h>
#include <select_port.h>
#include <stdio.h>
#include <stdlib.h>

struct scanned_ports {
    struct beluga_serial_ports *ports;
    size_t len;
};

struct scanned_ports ports;

static char *port_name_generator(const char *text, int state) {
    static size_t list_index, len;

    if (!state) {
        list_index = 0;
        len = strlen(text);
    }

    for (; list_index < ports.len; list_index++) {
        if (strncmp(ports.ports[list_index].port, text, len) == 0) {
            char *port = strdup(ports.ports[list_index].port);
            list_index++;
            return port;
        }
    }

    return NULL;
}

static char **port_completion(const char *text, int start, int end) {
    rl_attempted_completion_over = 1;
    return rl_completion_matches(text, port_name_generator);
}

static int select_port(void) {
    char *port;
    int index = -1;
    rl_completion_func_t *prev_func;
    int prev_completion_over;

    printf("Please select a serial port: \n");
    for (size_t i = 0; i < ports.len; i++) {
        printf("\t%s %s: %s\n", ports.ports[i].manufacturer,
               ports.ports[i].product, ports.ports[i].port);
    }

    prev_completion_over = rl_attempted_completion_over;
    prev_func = rl_attempted_completion_function;
    rl_attempted_completion_function = port_completion;
    port = readline("Port: ");

    for (size_t i = 0; i < ports.len; i++) {
        if (strcmp(port, ports.ports[i].port) == 0) {
            index = (int)i;
            break;
        }
    }
    rl_attempted_completion_function = prev_func;
    rl_attempted_completion_over = prev_completion_over;

    free(port);
    return index;
}

char *pick_port(void) {
    int port_index = -1;
    char *port;

    ports.ports = find_ports(&ports.len);

    if (ports.len == 0) {
        puts("No ports found");
        exit(EXIT_SUCCESS);
    }

    while (port_index < 0) {
        port_index = select_port();
    }

    port = calloc(strlen(ports.ports[port_index].port) + 1, sizeof(char));

    if (port == NULL) {
        perror("calloc()");
        exit(EXIT_FAILURE);
    }

    strcpy(port, ports.ports[port_index].port);
    cleanup_find_ports(&ports.ports, ports.len);

    return port;
}
