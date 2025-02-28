/**
 * @file ranging.h
 * @brief Ranging Module for UWB-based Distance Measurement System
 *
 * Implements the logic for the ranging, which supports both
 * single-sided and two-way ranging between nodes in a UWB (Ultra-Wideband)
 * network using the DW1000 chip. The module handles configuration,
 * initialization, and communication between nodes. It includes various
 * functions to configure and control the DW1000's parameters, such as data
 * rate, pulse rate, preamble length, and PAC size.
 *
 * Supports both the initiator and responder tasks, where the initiator
 * sends polling messages and the responder listens for them.
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 * @date 7/9/24
 */

#ifndef BELUGA_RANGING_H
#define BELUGA_RANGING_H

#include <deca_regs.h>
#include <serial/comms.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * Maximum transmission power register value
 */
#define TX_POWER_MAX 0x1F1F1F1F

/**
 * @brief Represents the channel delay values for the PGDELAY register.
 *
 * Defines the available channel delays in the PGDELAY register for controlling
 * the timing of signal propagation on different channels. These delays can be
 * used to fine-tune the performance of UWB communication.
 */
enum pgdelay_ch {
    ch1 = TC_PGDELAY_CH1, ///< Channel 1 delay
    ch2 = TC_PGDELAY_CH2, ///< Channel 2 delay
    ch3 = TC_PGDELAY_CH3, ///< Channel 3 delay
    ch4 = TC_PGDELAY_CH4, ///< Channel 4 delay
    ch5 = TC_PGDELAY_CH5, ///< Channel 5 delay
    ch7 = TC_PGDELAY_CH7, ///< Channel 7 delay
};

/**
 * @brief Represents the PHR (Physical Header) modes in UWB communication.
 *
 * Defines the available modes for the Physical Header (PHR) in UWB
 * communication, which determines the format and length of the PHR in the
 * transmitted frames.
 */
enum uwb_phr_mode {
    UWB_PHR_MODE_STD, ///< Standard PHR mode
    UWB_PWR_MODE_EXT  ///< Extended Frames PHR mode
};

/**
 * @brief Represents the data rates supported by the UWB module.
 *
 * Defines the available data rates for communication in the UWB
 * (Ultra-Wideband) network, where each data rate corresponds to a specific
 * speed of transmission between nodes.
 */
enum uwb_datarate {
    UWB_DR_6M8,  ///< 6.8 Mbps
    UWB_DR_850K, ///< 850 kbps
    UWB_DR_110K  ///< 110 kbps
};

/**
 * @brief Represents the Pulse Repetition Rate (PRR) options in UWB
 * communication.
 *
 * Defines the available pulse repetition rates for UWB communication,
 * which determine the frequency at which the pulses are transmitted.
 */
enum uwb_pulse_rate {
    UWB_PR_16M, ///< 16 MHz
    UWB_PR_64M  ///< 64 MHz
};

/**
 * @brief Represents the preamble lengths for UWB communication.
 *
 * Defines the available preamble lengths for Ultra-Wideband (UWB)
 * communication. The preamble length determines the duration of the preamble
 * used in the communication signal, which impacts the range and reliability
 * of the transmission.
 */
enum uwb_preamble_length {
    UWB_PRL_64 = 64,     ///< 64 symbols
    UWB_PRL_128 = 128,   ///< 128 symbols
    UWB_PRL_256 = 256,   ///< 256 symbols
    UWB_PRL_512 = 512,   ///< 512 symbols
    UWB_PRL_1024 = 1024, ///< 1024 symbols
    UWB_PRL_1536 = 1536, ///< 1536 symbols
    UWB_PRL_2048 = 2048, ///< 2048 symbols
    UWB_PRL_4096 = 4096, ///< 4096 symbols
    UWB_PRL_ERROR        ///< Invalid preamble length
};

/**
 * @brief Represents the Pulse Amplitude Code (PAC) sizes for UWB communication.
 *
 * Defines the available PAC sizes for Ultra-Wideband (UWB) communication.
 * The PAC size determines the number of samples in a single pulse, affecting
 * the accuracy and range of the distance measurement.
 */
enum uwb_pac {
    UWB_PAC8,  ///< 8 samples per pulse
    UWB_PAC16, ///< 16 samples per pulse
    UWB_PAC32, ///< 32 samples per pulse
    UWB_PAC64  ///< 64 samples per pulse
};

/**
 * @brief Represents the Start Frame Delimiter (SFD) types in UWB communication.
 *
 * This enum defines the available types of Start Frame Delimiters (SFD) used in
 * Ultra-Wideband (UWB) communication. The SFD is a marker that signals the
 * beginning of a frame in the communication protocol.
 */
enum uwb_sfd {
    UWB_STD_SFD, ///< Standard SFD
    UWB_NSTD_SFD ///< Nonstandard SFD
};

/**
 * @brief Prints the TX power in a non-standard (human readable) format
 * @param[in] tx_power The current TX power
 */
void print_tx_power(const struct comms *comms, uint32_t tx_power);

/**
 * @brief Prints the UWB data rate in a non-standard (human readable) format
 * @param[in] rate The current data rate
 * @return The data rate that was just printed
 */
enum uwb_datarate print_uwb_datarate(const struct comms *comms,
                                     enum uwb_datarate rate);

/**
 * @brief Prints the pulse rate in a non-standard (human readable) format
 * @param[in] rate The current pulse rate
 * @return The pulse rate
 */
enum uwb_pulse_rate print_pulse_rate(const struct comms *comms,
                                     enum uwb_pulse_rate rate);

/**
 * @brief Prints the PAC size in a non-standard (human readable) format
 * @param[in] pac The PAC size
 * @return The PAC size
 */
int32_t print_pac_size(const struct comms *comms, int32_t pac);

/**
 * @brief Prints the current PAN ID in a non-standard (human readable) format
 * @param[in] pan_id The PAN ID to print
 */
void print_pan_id(const struct comms *comms, uint32_t pan_id);

/**
 * @brief Sets the PHR mode for the DW1000
 * @param[in] mode The PHR mode to update
 * @return 0 upon success
 * @return -EINVAL if PHR mode is not valid
 */
int uwb_set_phr_mode(enum uwb_phr_mode mode);

/**
 * @brief Sets the data rate of the DW1000
 * @param[in] rate The new data rate of the DW1000
 * @return 0 upon success
 * @return -EINVAL if data rate is an invalid value
 * @return -EBUSY if UWB is active
 */
int uwb_set_datarate(enum uwb_datarate rate);

/**
 * @brief Sets the DW1000 pulse rate
 * @param[in] rate The pulse rate to of the DW1000
 * @return 0 upon success
 * @return -EINVAL if rate is invalid
 * @return -EBUSY if UWB is active
 */
int uwb_set_pulse_rate(enum uwb_pulse_rate rate);

/**
 * @brief Sets the preamble length of the DW1000
 * @param[in] length The new preamble length of the DW1000
 * @return 0 upon success
 * @return -EINVAL if length is invalid
 * @return -EBUSY if UWB is active
 */
int uwb_set_preamble(enum uwb_preamble_length length);

/**
 * @brief Sets the PAC size of the DW1000
 * @param[in] pac The new PAC size of the DW1000
 * @return 0 upon success
 * @return -EINVAL if PAC size is invalid
 * @return -EBUSY if UWB is active
 */
int set_pac_size(enum uwb_pac pac);

/**
 * @brief Sets the SFD mode of the DW1000
 * @param[in] mode The new SFD mode
 * @return 0 upon success
 * @return -EINVAL upon failure
 * @return -EBUSY if UWB is active
 */
int set_sfd_mode(enum uwb_sfd mode);

/**
 * @brief Sets the UWB channel to use for the DW1000
 * @param[in] channel The new UWB channel
 * @return 0 upon success
 * @return -EINVAL if invalid channel'
 * @return -EBUSY if UWB is active
 */
int set_uwb_channel(uint32_t channel);

/**
 * Sets the ranging mode used by the initiator and responder
 * @param[in] value The ranging mode. If `true`, use two-way ranging, if
 * `false`, use single-sided ranging
 */
void set_twr_mode(bool value);

/**
 * @brief Set the rate the initiator sends polling messages
 * @param[in] rate The new rate the initiator sends polling messages. If 0, then
 * do not send polling messages.
 * @return 0 upon success
 * @return -EINVAL if rate is an invalid value
 */
int set_rate(uint32_t rate);

/**
 * @brief Sets the transmit power of the DW1000
 * @param[in] tx_power The new transmit power of the DW1000
 */
void set_tx_power(uint32_t tx_power);

/**
 * @brief Initialize the DW1000 for ranging.
 *
 * Wakes the DW1000 (if coming out of deep sleep), resets the DW1000, and
 * initializes the DW1000 with default values
 */
void init_uwb(void);

/**
 * @brief Creates the ranging thread and initiates its data
 */
void init_ranging_thread(void);

/**
 * @brief Creates the responder thread and initiates its data
 */
void init_responder_thread(void);

#endif // BELUGA_RANGING_H
