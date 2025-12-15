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
#include <stdio.h>

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

    size_t len;
    struct beluga_serial_ports *ports = find_ports(&len);

    printf("Found %lu ports\n", len);

    for (size_t i = 0; i < len; i++) {
        printf("%s %s: %s\n", ports[i].manufacturer, ports[i].product,
               ports[i].port);
    }

    cleanup_find_ports(&ports, len);

    return 0;
}
