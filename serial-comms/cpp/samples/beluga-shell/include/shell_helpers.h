/**
 * @file shell_helpers.h
 *
 * @brief
 *
 * @date 12/15/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_SHELL_HELPERS_H
#define BELUGA_SHELL_HELPERS_H

typedef void signal_handler_t(int);
signal_handler_t *Signal(int signum, signal_handler_t *handler);

#endif // BELUGA_SHELL_HELPERS_H