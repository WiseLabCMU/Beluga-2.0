/**
 * @file sio.c
 *
 * @brief
 *
 * @date 12/16/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <errno.h>
#include <sio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void reverse(char *s, size_t len) {
    size_t i, j;
    for (i = 0, j = len - 1; i < j; i++, j++) {
        char c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

static size_t write_digits(uintmax_t v, char *s, unsigned char b) {
    size_t i = 0;
    do {
        unsigned char c = v % b;
        if (c < 10) {
            s[i++] = (char)(c + '0');
        } else {
            s[i++] = (char)(c - 10 + 'a');
        }
    } while ((v /= b) > 0);

    return i;
}

static size_t intmax_to_string(intmax_t v, char *s, unsigned char b) {
    size_t len;

    if (v < 0) {
        len = write_digits((uintmax_t)-v, s, b);
        s[len++] = '-';
    } else {
        len = write_digits(v, s, b);
    }

    s[len] = '\0';
    reverse(s, len);
    return len;
}

static size_t uintmax_to_string(uintmax_t v, char *s, unsigned char b) {
    size_t len = write_digits(v, s, b);
    s[len] = '\0';
    reverse(s, len);
    return len;
}

ssize_t sio_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    ssize_t ret = sio_vdprintf(STDOUT_FILENO, fmt, args);
    va_end(args);
    return ret;
}

ssize_t sio_dprintf(int fileno, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    ssize_t ret = sio_vdprintf(fileno, fmt, args);
    va_end(args);
    return ret;
}

ssize_t sio_eprintf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    ssize_t ret = sio_vdprintf(STDERR_FILENO, fmt, args);
    va_end(args);
    return ret;
}

struct format_data {
    const char *str;
    size_t len;
    char buf[512];
};

static size_t _handle_format(const char *fmt, va_list argp,
                             struct format_data *data) {
    size_t pos = 0;
    bool handled = false;

    if (fmt[0] == '%') {
        char convert_type = '\0';
        union {
            uintmax_t u;
            intmax_t s;
        } convert_val = {.u = 0};

        switch (fmt[1]) {
        case 'c': {
            data->buf[0] = (char)va_arg(argp, int);
            data->buf[1] = '\0';
            data->str = data->buf;
            data->len = 1;
            handled = true;
            pos += 2;
            break;
        }
        case 's': {
            data->str = va_arg(argp, char *);
            if (data->str == NULL) {
                data->str = "(null)";
            }
            data->len = strlen(data->str);
            handled = true;
            pos += 2;
            break;
        }
        case '%': {
            data->str = fmt;
            data->len = 1;
            handled = true;
            pos += 2;
            break;
        }
        case 'p': {
            void *ptr = va_arg(argp, void *);
            if (ptr == NULL) {
                data->str = "(nil)";
                data->len = strlen(data->str);
                handled = true;
            } else {
                convert_type = 'p';
                convert_val.u = (uintmax_t)(uintptr_t)ptr;
            }
            pos += 2;
            break;
        }
        case 'd':
        case 'i': {
            convert_type = 'd';
            convert_val.s = (intmax_t)va_arg(argp, int);
            pos += 2;
            break;
        }
        case 'u':
        case 'x':
        case 'o': {
            convert_type = fmt[1];
            convert_val.u = (uintmax_t)va_arg(argp, unsigned);
            pos += 2;
            break;
        }

        case 'l': {
            switch (fmt[2]) {
            case 'd':
            case 'i': {
                convert_type = 'd';
                convert_val.s = (intmax_t)va_arg(argp, long);
                pos += 3;
                break;
            }
            case 'u':
            case 'x':
            case 'o': {
                convert_type = fmt[2];
                convert_val.u = (uintmax_t)va_arg(argp, unsigned long);
                pos += 3;
                break;
            }
            }
            break;
        }
        case 'z': {
            switch (fmt[2]) {
            case 'd':
            case 'i': {
                convert_type = 'd';
                convert_val.s = (intmax_t)(uintmax_t)va_arg(argp, size_t);
                pos += 3;
                break;
            }
            case 'u':
            case 'x':
            case 'o': {
                convert_type = fmt[2];
                convert_val.u = (uintmax_t)va_arg(argp, size_t);
                pos += 3;
                break;
            }
            }
            break;
        }
        }

        switch (convert_type) {
        case 'd': {
            data->str = data->buf;
            data->len = intmax_to_string(convert_val.s, data->buf, 10);
            handled = true;
            break;
        }
        case 'u': {
            data->str = data->buf;
            data->len = uintmax_to_string(convert_val.u, data->buf, 10);
            handled = true;
            break;
        }
        case 'x': {
            data->str = data->buf;
            data->len = uintmax_to_string(convert_val.u, data->buf, 16);
            handled = true;
            break;
        }
        case 'o': {
            data->str = data->buf;
            data->len = uintmax_to_string(convert_val.u, data->buf, 8);
            handled = true;
            break;
        }
        case 'p': {
            strcpy(data->buf, "0x");
            data->str = data->buf;
            data->len = uintmax_to_string(convert_val.u, data->buf + 2, 16) + 2;
            handled = true;
            break;
        }
        }
    }

    if (!handled) {
        data->str = fmt;
        data->len = 1 + strcspn(fmt + 1, "%");
        pos += data->len;
    }

    return pos;
}

static ssize_t writen(int fd, const void *usrbuf, size_t n) {
    size_t nleft = n;
    size_t nwritten;
    const char *bufp = usrbuf;

    while (nleft > 0) {
        if ((nwritten = write(fd, bufp, nleft)) <= 0) {
            if (errno != EINTR) {
                return -1;
            }

            nwritten = 0;
        }
        nleft -= (size_t)nwritten;
        bufp += nwritten;
    }
    return (ssize_t)n;
}

ssize_t sio_vdprintf(int fileno, const char *format, va_list argp) {
    size_t pos = 0;
    ssize_t num_written = 0;
    struct format_data data;

    while (format[pos] != '\0') {
        memset(&data, 0, sizeof(data));

        pos += _handle_format(&format[pos], argp, &data);

        if (data.len > 0) {
            ssize_t ret = writen(fileno, (const void *)data.str, data.len);
            if (ret < 0 || (size_t)ret != data.len) {
                return -1;
            }
        }
    }

    return num_written;
}

void __sio_assert_fail(const char *assertion, const char *file,
                       unsigned int line, const char *function) {
    sio_dprintf(STDERR_FILENO, "%s:%u: %s: Assertion `%s` failed.\n", file,
                line, function, assertion);
    abort();
}
