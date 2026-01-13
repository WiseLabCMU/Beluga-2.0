/**
 * @file autocomplete.h
 *
 * @brief
 *
 * @date 12/18/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_AUTOCOMPLETE_H
#define BELUGA_AUTOCOMPLETE_H

int initialize_autocomplete(void);
void cleanup_autocomplete(void);
int autocomplete_register_builtin_command(const char *command);
const char *command_path(const char *command);

#endif // BELUGA_AUTOCOMPLETE_H