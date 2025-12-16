/**
 * @file sio.h
 *
 * @brief
 *
 * @date 12/16/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_SIO_H
#define BELUGA_SIO_H

#include <stdarg.h>
#include <sys/types.h>

ssize_t sio_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
ssize_t sio_dprintf(int fileno, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
ssize_t sio_eprintf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
ssize_t sio_vdprintf(int fileno, const char *format, va_list argp)
    __attribute__((format(printf, 2, 0)));

#if defined(NDEBUG)
#define sio_assert(expr)                                                       \
    ((expr) ? (void)0 : __sio_assert_fail(#expr, __FILE__, __LINE__, __func__))
#else
#define sio_assert(expr) (void)0
#endif

void __sio_assert_fail(const char *assertion, const char *file,
                       unsigned int line, const char *function)
    __attribute__((noreturn));

#endif // BELUGA_SIO_H