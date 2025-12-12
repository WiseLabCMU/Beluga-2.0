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
    struct beluga_serial_attr attr = {.port = "/dev/ttyACM1",
                                      .baud = BAUD_115200};

    struct beluga_serial *serial = create_beluga_serial_instance(&attr);

    printf("Object: %p\nClass Instance: %p\n", serial, serial->ctx);

    destroy_beluga_serial_instance(&serial);

    return 0;
}
