/**
 * @file utils.h
 * @brief Utility macros
 * @date 6/19/24
 * @author Tom Schmitz
 */

#ifndef BELUGA_UTILS_H
#define BELUGA_UTILS_H

#include <zephyr/sys/util.h>

/**
 * Marks a function or variable as unused
 */
#define UNUSED __attribute__((unused))

/**
 * Marks a local function for inlining
 */
#define STATIC_INLINE __attribute__((always_inline)) static inline

/**
 * Internal macro for deprecating a function or variable without a message
 */
#define _DEPRECATED __attribute__((deprecated))

/**
 * Internal macro for deprecating a function or variable with a message
 *
 * @param[in] MSG The deprecation message
 */
#define _DEPRECATED_MSG(MSG) __attribute__((dprecated(MSG)))

/**
 * Marks a function or variable as deprecated with an optional message as to why
 * it is deprecated
 *
 * @param[in] msg The deprecation message
 */
#define DEPRECATED(msg...)                                                     \
    COND_CODE_1(IS_EMPTY(msg), (_DEPRECATED),                                  \
                (_DEPRECATED_MSG(GET_ARG_N(1, msg))))

/**
 * Indicates that a function does not return
 */
#define NO_RETURN __attribute__((noreturn))

#if defined(__GNUC__) && __GNUC__ >= 7
#define FALLTHROUGH __attribute__((fallthrough))
#else
#define FALLTHROUGH ((void)0)
#endif // defined(__GNUC__) && __GNUC__ >= 7

/**
 * Increments a variable and resets it back to 0 if it reaches the upper bound
 *
 * @param[in] x The variable to increment
 * @param[in] upper_bound The non-inclusive maximum value of the variable
 *
 * @note When using this, the following condition will always be true for all
 * positive values: `x < upper_bound`
 */
#define BOUND_INCREMENT(x, upper_bound) x = ((x) + 1) % (upper_bound)

#endif // BELUGA_UTILS_H
