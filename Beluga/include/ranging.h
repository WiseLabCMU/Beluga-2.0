//
// Created by tom on 7/9/24.
//

#ifndef BELUGA_RANGING_H
#define BELUGA_RANGING_H

#include <deca_regs.h>
#include <stdbool.h>
#include <stdint.h>

/* Maximum transmission power register value */
#define TX_POWER_MAX 0x1F1F1F1F

enum pgdelay_ch {
    ch1 = TC_PGDELAY_CH1,
    ch2 = TC_PGDELAY_CH2,
    ch3 = TC_PGDELAY_CH3,
    ch4 = TC_PGDELAY_CH4,
    ch5 = TC_PGDELAY_CH5,
    ch7 = TC_PGDELAY_CH7,
};

enum uwb_phr_mode { UWB_PHR_MODE_STD, UWB_PWR_MODE_EXT };

enum uwb_datarate { UWB_DR_6M8, UWB_DR_850K, UWB_DR_110K };

enum uwb_pulse_rate { UWB_PR_16M, UWB_PR_64M };

enum uwb_preamble_length {
    UWB_PRL_64 = 64,
    UWB_PRL_128 = 128,
    UWB_PRL_256 = 256,
    UWB_PRL_512 = 512,
    UWB_PRL_1024 = 1024,
    UWB_PRL_1536 = 1536,
    UWB_PRL_2048 = 2048,
    UWB_PRL_4096 = 4096,
    UWB_PRL_ERROR
};

enum uwb_pac { UWB_PAC8, UWB_PAC16, UWB_PAC32, UWB_PAC64 };

enum uwb_sfd { UWB_STD_SFD, UWB_NSTD_SFD };

int uwb_set_phr_mode(enum uwb_phr_mode mode);
int uwb_set_datarate(enum uwb_datarate rate);
int uwb_set_pulse_rate(enum uwb_pulse_rate rate);
int uwb_set_preamble(enum uwb_preamble_length length);
int set_pac_size(enum uwb_pac pac);
int set_sfd_mode(enum uwb_sfd mode);
int set_uwb_channel(uint32_t channel);

void set_twr_mode(bool value);
bool get_twr_mode(void);
void set_rate(uint32_t rate);
uint32_t get_rate(void);
void set_tx_power(uint32_t tx_power);

void init_uwb(void);

void init_ranging_thread(void);
void init_responder_thread(void);

#endif // BELUGA_RANGING_H
