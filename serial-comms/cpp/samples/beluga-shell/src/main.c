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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

static int select_port(struct beluga_serial_ports *ports, size_t len) {
    char *port;
    int index = -1;

    printf("Please select a serial port: \n");
    for (size_t i = 0; i < len; i++) {
        printf("\t%s %s: %s\n", ports[i].manufacturer, ports[i].product,
               ports[i].port);
    }

    // TODO: Set up autocomplete here
    port = readline("Port: ");

    for (size_t i = 0; i < len; i++) {
        if (strcmp(port, ports[i].port) == 0) {
            index = (int)i;
            break;
        }
    }

    free(port);
    return index;
}

int main(int argc, char *argv[]) {
    // struct beluga_serial_attr attr = {.port = "/dev/ttyACM1",
    //                                   .baud = BAUD_115200,
    //                                   .timeout = 2000,
    //                                   .serial_timeout = 100};
    //
    // struct beluga_serial *serial = create_beluga_serial_instance(&attr);
    //
    // beluga_serial_start(serial);
    //
    // printf("Error: %d\n", beluga_serial_swap_port(serial, "/dev/ttyACM"));
    //
    // printf("Response: %s\n", serial->response);
    //
    // beluga_serial_stop(serial);
    //
    // destroy_beluga_serial_instance(&serial);

    int port_index = -1;
    size_t len;
    char *port;
    struct beluga_serial_ports *ports = find_ports(&len);
    struct beluga_serial_attr attr = {
        .baud = BAUD_115200, .timeout = 2000, .serial_timeout = 100};
    struct beluga_serial *serial;

    if (len == 0) {
        printf("No ports found\n");
        return 0;
    }

    while (port_index < 0) {
        port_index = select_port(ports, len);
    }

    port = calloc(strlen(ports[port_index].port) + 1, sizeof(char));

    if (port == NULL) {
        perror("Malloc");
        return -errno;
    }

    strcpy(port, ports[port_index].port);

    cleanup_find_ports(&ports, len);

    attr.port = port;

    serial = create_beluga_serial_instance(&attr);

    if (serial == NULL) {
        perror("create_beluga_serial_instance()");
        return -errno;
    }

    beluga_serial_start(serial);
    beluga_serial_id(serial, NULL);

    printf("Response: %s\n", serial->response);

    destroy_beluga_serial_instance(&serial);
    free(port);

    return 0;
}
