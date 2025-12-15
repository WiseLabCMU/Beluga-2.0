/**
 * @file shell_helpers.c
 *
 * @brief
 *
 * @date 12/15/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <shell_helpers.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

signal_handler_t *Signal(int signum, signal_handler_t handler) {
    struct sigaction action, old_action;

    action.sa_handler = handler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_RESTART;

    if (sigaction(signum, &action, &old_action) < 0) {
        perror("Signal error");
        exit(EXIT_FAILURE);
    }

    return old_action.sa_handler;
}
