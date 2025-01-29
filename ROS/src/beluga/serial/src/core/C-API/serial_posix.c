/**
 * @file posix_serial.c
 *
 * @brief
 *
 * @date 1/28/25
 *
 * @author tom
 */

#include <errno.h>
#include <fcntl.h>
#include <serial/core/C-API/serial_common.h>
#include <serial/core/C-API/serial_posix.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

int open_port(const char *port) {
    return open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
}

static int set_baud(struct termios *tty, enum BaudRate baudrate) {
    speed_t baud;

    switch (baudrate) {
    case BAUD_0:
        baud = B0;
        break;
    case BAUD_50:
        baud = B50;
        break;
    case BAUD_75:
        baud = B75;
        break;
    case BAUD_110:
        baud = B110;
        break;
    case BAUD_134:
        baud = B134;
        break;
    case BAUD_150:
        baud = B150;
        break;
    case BAUD_200:
        baud = B200;
        break;
    case BAUD_300:
        baud = B300;
        break;
    case BAUD_600:
        baud = B600;
        break;
    case BAUD_1200:
        baud = B1200;
        break;
    case BAUD_1800:
        baud = B1800;
        break;
    case BAUD_2400:
        baud = B2400;
        break;
    case BAUD_4800:
        baud = B4800;
        break;
    case BAUD_9600:
        baud = B9600;
        break;
    case BAUD_19200:
        baud = B19200;
        break;
    case BAUD_38400:
        baud = B38400;
        break;
    case BAUD_57600:
        baud = B57600;
        break;
    case BAUD_115200:
        baud = B115200;
        break;
    case BAUD_230400:
        baud = B230400;
        break;
    case BAUD_460800:
        baud = B460800;
        break;
    case BAUD_500000:
        baud = B500000;
        break;
    case BAUD_576000:
        baud = B576000;
        break;
    case BAUD_921600:
        baud = B921600;
        break;
    case BAUD_1000000:
        baud = B1000000;
        break;
    case BAUD_1152000:
        baud = B1152000;
        break;
    case BAUD_1500000:
        baud = B1500000;
        break;
    case BAUD_2000000:
        baud = B2000000;
        break;
    case BAUD_2500000:
        baud = B2500000;
        break;
    case BAUD_3000000:
        baud = B3000000;
        break;
    case BAUD_3500000:
        baud = B3500000;
        break;
    case BAUD_4000000:
        baud = B4000000;
        break;
    default:
        return -EINVAL;
    }

    cfsetispeed(tty, baud);
    cfsetospeed(tty, baud);
    return 0;
}

static int set_parity(struct termios *tty, enum Parity parity) {
    switch (parity) {
    case PARITY_NONE:
        tty->c_cflag &= ~PARENB;
        break;
    case PARITY_EVEN:
        tty->c_cflag |= PARENB;
        tty->c_cflag &= ~PARODD;
        break;
    case PARITY_ODD:
        tty->c_cflag |= (PARENB | PARODD);
        break;
    case PARITY_MARK:
        tty->c_cflag |= (PARENB | CMSPAR | PARODD);
        break;
    case PARITY_SPACE:
        tty->c_cflag |= (PARENB | CMSPAR);
        tty->c_cflag &= ~PARODD;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int set_stopbits(struct termios *tty, enum StopBits bits) {
    switch (bits) {
    case STOPBITS_1:
        tty->c_cflag &= ~CSTOPB;
    case STOPBITS_1P5:
    case STOPBITS_2:
        tty->c_cflag |= CSTOPB;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int set_bytesize(struct termios *tty, enum ByteSize size) {
    tty->c_cflag &= ~CSIZE;
    switch (size) {
    case SIZE_5:
        tty->c_cflag |= CS5;
        break;
    case SIZE_6:
        tty->c_cflag |= CS6;
        break;
    case SIZE_7:
        tty->c_cflag |= CS7;
        break;
    case SIZE_8:
        tty->c_cflag |= CS8;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

int configure_port(struct serial_posix_config *config) {
    struct termios tty;
    int ret;
    if (config == NULL) {
        return -EINVAL;
    }

    if (tcgetattr(config->fd, &tty) != 0) {
        return -errno;
    }

    ret = set_baud(&tty, config->baudrate);
    if (ret < 0) {
        return ret;
    }

    ret = set_parity(&tty, config->parity);
    if (ret < 0) {
        return ret;
    }

    ret = set_stopbits(&tty, config->stopbits);
    if (ret < 0) {
        return ret;
    }

    ret = set_bytesize(&tty, config->bytesize);
    if (ret < 0) {
        return ret;
    }

    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(config->fd, TCSANOW, &tty) != 0) {
        return -errno;
    }

    return 0;
}

ssize_t read_port(int fd, uint8_t *buf, size_t nbytes,
                  struct timeval *time_left) {
    fd_set read_set;
    int ret;
    FD_ZERO(&read_set);
    FD_SET(fd, &read_set);

    ret = select(fd + 1, &read_set, NULL, NULL, time_left);
    if (ret == 0) {
        errno = ETIMEDOUT;
        return -1;
    }

    return read(fd, buf, nbytes);
}

ssize_t write_port(int fd, const uint8_t *buf, size_t nbytes) {
    return write(fd, buf, nbytes);
}

int close_port(int fd) { return close(fd); }

ssize_t port_in_waiting(int fd) {
    ssize_t waiting = 0;
    if (ioctl(fd, TIOCINQ, &waiting) < 0) {
        return -errno;
    }
    return waiting;
}

int select_write_port(int fd, struct timeval *timeout) {
    fd_set write_set;
    FD_ZERO(&write_set);
    FD_SET(fd, &write_set);
    return select(fd + 1, NULL, &write_set, NULL, timeout);
}

int port_flush(int fd) { return tcdrain(fd); }

int port_reset_input(int fd) { return tcflush(fd, TCIFLUSH); }

int port_reset_output(int fd) { return tcflush(fd, TCOFLUSH); }

int port_cts(int fd) {
    unsigned int status = 0;
    int ret = ioctl(fd, TIOCMGET, &status);
    if (ret < 0) {
        return ret;
    }
    return (status & TIOCM_CTS) != 0;
}

int port_dsr(int fd) {
    unsigned int status = 0;
    int ret = ioctl(fd, TIOCMGET, &status);
    if (ret < 0) {
        return ret;
    }
    return (status & TIOCM_DSR) != 0;
}

int port_ri(int fd) {
    unsigned int status = 0;
    int ret = ioctl(fd, TIOCMGET, &status);
    if (ret < 0) {
        return ret;
    }
    return (status & TIOCM_RI) != 0;
}

int port_cd(int fd) {
    unsigned int status = 0;
    int ret = ioctl(fd, TIOCMGET, &status);
    if (ret < 0) {
        return ret;
    }
    return (status & TIOCM_CD) != 0;
}

int port_set_rts_state(int fd, bool state) {
    unsigned int status = TIOCM_RTS;
    int ret;
    if (state) {
        ret = ioctl(fd, TIOCMBIS, &status);
    } else {
        ret = ioctl(fd, TIOCMBIC, &status);
    }
    return ret;
}

int port_set_dtr_state(int fd, bool state) {
    unsigned int status = TIOCM_DTR;
    int ret;
    if (state) {
        ret = ioctl(fd, TIOCMBIS, &status);
    } else {
        ret = ioctl(fd, TIOCMBIC, &status);
    }
    return ret;
}
