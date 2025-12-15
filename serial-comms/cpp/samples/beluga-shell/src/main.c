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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {
    char *port;
    struct beluga_serial_attr attr = {
        .baud = BAUD_115200, .timeout = 2000, .serial_timeout = 100};
    struct beluga_serial *serial;

    port = pick_port();
    attr.port = port;

    serial = create_beluga_serial_instance(&attr);

    if (serial == NULL) {
        perror("create_beluga_serial_instance()");
        return -errno;
    }

    beluga_serial_start(serial);

    // TODO: Figure out where to free these
    destroy_beluga_serial_instance(&serial);
    free(port);

    return 0;
}
