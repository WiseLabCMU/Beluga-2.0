/**
 * @file utils.h
 *
 * @brief
 *
 * @date 9/5/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_UTILS_H
#define BELUGA_UTILS_H

extern "C" {

#define COND_CODE_0(_flag, _if_0_code, _else_code)                             \
    Z_COND_CODE_0(_flag, _if_0_code, _else_code)

#define Z_COND_CODE_0(_flag, _if_0_code, _else_code)                           \
    __COND_CODE(_ZZZZ##_flag, _if_0_code, _else_code)

#define _ZZZZ0 _YYYY,

#define __COND_CODE(one_or_two_args, _if_code, _else_code)                     \
    __GET_ARG2_DEBRACKET(one_or_two_args _if_code, _else_code)

#define __GET_ARG2_DEBRACKET(ignore_this, val, ...) __DEBRACKET val

#define __DEBRACKET(...)                            __VA_ARGS__

#define IS_EMPTY(...)                               Z_IS_EMPTY_(__VA_ARGS__)

#define Z_IS_EMPTY_(...)                                                       \
    Z_IS_EMPTY__(Z_HAS_COMMA(__VA_ARGS__),                                     \
                 Z_HAS_COMMA(Z_TRIGGER_PARENTHESIS_ __VA_ARGS__),              \
                 Z_HAS_COMMA(__VA_ARGS__(/*empty*/)),                          \
                 Z_HAS_COMMA(Z_TRIGGER_PARENTHESIS_ __VA_ARGS__(/*empty*/)))

#define Z_HAS_COMMA(...)                                                       \
    NUM_VA_ARGS_LESS_1_IMPL(                                                   \
        __VA_ARGS__, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  \
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,   \
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0)

#define Z_TRIGGER_PARENTHESIS_(...) ,

#define NUM_VA_ARGS_LESS_1_IMPL(                                               \
    _ignored, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, \
    _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, \
    _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, \
    _45, _46, _47, _48, _49, _50, _51, _52, _53, _54, _55, _56, _57, _58, _59, \
    _60, _61, _62, N, ...)                                                     \
    N

#define Z_IS_EMPTY__(_0, _1, _2, _3)                                           \
    Z_HAS_COMMA(Z_CAT5(Z_IS_EMPTY_CASE_, _0, _1, _2, _3))

#define Z_CAT5(_0, _1, _2, _3, _4) _0##_1##_2##_3##_4

#define Z_IS_EMPTY_CASE_0001       ,
};

#endif // BELUGA_UTILS_H
