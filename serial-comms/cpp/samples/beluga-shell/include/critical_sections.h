/**
 * @file critical_sections.h
 *
 * @brief
 *
 * @date 12/19/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_CRITICAL_SECTIONS_H
#define BELUGA_CRITICAL_SECTIONS_H

#include <signal.h>
#include <sio.h>
#include <stddef.h>

static __attribute__((always_inline)) inline void sig_cleanup(int *key) {
    sio_assert(*key == 1);
}

#define SIGNAL_HANDLER_CRIT_SECTION_ONEXIT __attribute__((cleanup(sig_cleanup)))

#define SIGNAL_HANDLER_CRIT_SECTION()                                          \
    int __prev_errno = errno;                                                  \
    sigset_t __mask, __prev;                                                   \
    sigfillset(&__mask);                                                       \
    sigprocmask(SIG_BLOCK, &__mask, &__prev);                                  \
    for (int __key SIGNAL_HANDLER_CRIT_SECTION_ONEXIT = 0; !__key;             \
         sigprocmask(SIG_UNBLOCK, &__prev, NULL), errno = __prev_errno,        \
                   __key = 1)

static __attribute__((always_inline)) inline int block_signals(sigset_t *mask,
                                                               sigset_t *prev) {
    sigemptyset(mask);
    sigaddset(mask, SIGCHLD);
    sigaddset(mask, SIGINT);
    sigaddset(mask, SIGTSTP);
    sigprocmask(SIG_BLOCK, mask, prev);
    return 0;
}

static __attribute__((always_inline)) inline void
unblock_signals(sigset_t *mask) {
    sigprocmask(SIG_SETMASK, mask, NULL);
}

static __attribute__((always_inline)) inline void
crit_section_onexit(unsigned int *key) {
    sio_assert(*key);
}
#define CRITICAL_SECTION_ONEXIT __attribute__((cleanup(crit_section_onexit)))
#define CRITICAL_SECTION_BREAK  continue
#define CRITICAL_SECTION_EXIT_FUNCTION(prev_, stmt_, ret_)                     \
    do {                                                                       \
        __i = 1;                                                               \
        unblock_signals(&prev_);                                               \
        stmt_;                                                                 \
        return ret_;                                                           \
    } while (0)

#define CRITICAL_SECTION(mask_, prev_)                                         \
    for (unsigned int __i CRITICAL_SECTION_ONEXIT =                            \
             block_signals(&mask_, &prev_);                                    \
         !__i; unblock_signals(&prev_), __i = 1)

#endif // BELUGA_CRITICAL_SECTIONS_H