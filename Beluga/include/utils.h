//
// Created by tom on 6/19/24.
//

#ifndef BELUGA_UTILS_H
#define BELUGA_UTILS_H

#define UNUSED                          __attribute__((unused))
#define STATIC_INLINE                   __attribute__((always_inline)) static inline
#define DEPRECATED(MSG)                 __attribute__((deprecated(MSG)))
#define NO_RETURN                       __attribute__((noreturn))

#define BOUND_INCREMENT(x, upper_bound) x = ((x) + 1) % (upper_bound)

#endif // BELUGA_UTILS_H
