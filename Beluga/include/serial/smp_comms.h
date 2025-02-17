/**
 * @file smp_comms.h
 *
 * @brief
 *
 * @date 2/17/25
 *
 * @author tom
 */

#ifndef BELUGA_DTS_SMP_COMMS_H
#define BELUGA_DTS_SMP_COMMS_H

#include <zephyr/kernel.h>

#define SMP_COMMS_RX_BUF_SIZE 255

struct smp_comms_data {
    struct net_buf_pool *buf_pool;
    struct k_fifo buf_ready;
    struct net_buf *buf;
    atomic_t esc_state;
};

size_t smp_comms_rx_bytes(struct smp_comms_data *data, const uint8_t *bytes, size_t size);
void smp_comms_process(struct smp_comms_data *data);
int smp_comms_init(void);

#endif //BELUGA_DTS_SMP_COMMS_H
