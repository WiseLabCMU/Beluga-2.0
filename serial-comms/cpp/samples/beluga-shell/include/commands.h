/**
 * @file commands.h
 *
 * @brief
 *
 * @date 12/17/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_COMMANDS_H
#define BELUGA_COMMANDS_H

#include <beluga_serial_c_api.h>

int initialize_builtin_commands(struct beluga_serial *serial);
void report_unexpected_reboot(void);
void report_fatal_error(const char *msg);

#endif // BELUGA_COMMANDS_H