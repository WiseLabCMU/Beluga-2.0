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
void report_range_event(const struct range_event *event);
void report_neighbor_update(const struct beluga_neighbor *updates, size_t len);
void report_range_update(const struct beluga_neighbor *updates, size_t len);

#endif // BELUGA_COMMANDS_H