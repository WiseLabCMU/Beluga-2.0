## Overview

Beluga can be controlled through the AT command interface. There are 3 types of commands:

1. Configuration commands
2. Control commands
3. Status commands

???+ info

    **Configuration Commands:** Commands that configure Beluga for different applications. These settings persist. 
    Passing in no value reads out the current configuration.

    **Control Commands:**: Commands that control the state of Beluga. These commands do not persist over reboots.

    **Status Commands:** Commands that retrieve the current state of Beluga.

## Configuration Commands

### AT+ID
Sets or retrieves the node ID of the Beluga modem.

```shell title="Usage"
AT+ID
AT+ID <node id>
```

| Argument  | Description                                                          |
|-----------|----------------------------------------------------------------------|
| `node id` | The node ID of the modem. This must be a positive, non-zero integer. |

!!! note

    Each modem on the network must have a unique ID.

### AT+BOOTMODE

Determines how the node should behave when it reboots or powers on.

```shell title="Usage"
AT+BOOTMODE
AT+BOOTMODE <mode>
```

=== "Argument Descriptions"

    | Argument | Description                                             |
    |:---------|:--------------------------------------------------------|
    | mode     | The state to place Beluga in when rebooting/powering on |

=== "Mode Descriptions"

    | Value | Description                                       |
    |:------|:--------------------------------------------------|
    | `0`   | (Default) Do nothing on startup (BLE and UWB off) |
    | `1`   | Start BLE advertising and scanning on startup     |
    | `2`   | Start BLE and UWB on startup; fully functional    |

### AT+RATE

Determines the rate that the initiator will send ranging requests to nearby neighbors.

```shell title="Usage"
AT+RATE
AT+RATE <period>
```

| Argument | Description                                                                                 |
|:---------|:--------------------------------------------------------------------------------------------|
| `period` | The rate (in milliseconds) that the node initiates ranging exchanges with neighboring nodes |

??? note "Turning off the initiator"

    Setting the `period` to `0` will configure the node in "responder" only mode. In other words,
    the node will not initiate ranging exchanges and will only respond to ranging requests.

??? info "Maximum value for period"

    The maximum period is a compile-time configuration. See \<todo: insert link or redirect here\>.

### AT+CHANNEL

Determines which UWB channel to use for ranging.

```shell title="Usage"
AT+CHANNEL
AT+CHANNEL <channel>
```

=== "Argument Descriptions"

    | Argument  | Description                              |
    |:----------|:-----------------------------------------|
    | `channel` | The channel to use for ranging exchanges |

=== "Channel Descriptions"

    | Value | Description                                 |
    |:------|:--------------------------------------------|
    | `1`   | f~c~ = 2494.4 MHz, BW = 499.2 MHz           |
    | `2`   | f~c~ = 3993.6 MHz, BW = 499.2 MHz           |
    | `3`   | f~c~ = 4492.8 MHz, BW = 499.2 MHz           |
    | `4`   | f~c~ = 3993.6 MHz, BW = 1331.2 MHz[^1]      |
    | `5`   | (Default) f~c~ = 6489.6 MHz, BW = 499.2 MHz |
    | `7`   | f~c~ = 6489.6 MHz, BW = 1081.6 MHz[^1]      |

    [^1]: The DW1000 has a maximum receive bandwidth of 900 MHz


???+ warning

    Updating this parameter also has an affect on the BLE advertising data. Nodes with mismatching
    data will not attempt to range to each other.

### AT+RESET

Resets the Beluga configurations back to their default values.

```shell title="Usage"
AT+RESET
```

??? note

    See all configuration commands for their default values. This will set the node ID to `0`, which
    is considered an invalid value.

### AT+TIMEOUT

Sets the amount of time (in milliseconds) that a neighboring node can stay within the neighbor list
without any updates.

```shell title="Usage"
AT+TIMEOUT
AT+TIMEOUT <timeout>
```

| Argument  | Description                                                                                                                            |
|:----------|:---------------------------------------------------------------------------------------------------------------------------------------|
| `timeout` | The amount of time a neighbor node can stay in the neighbor list without any updates. This must be a positive integer. (Default: 9000) |

### AT+TXPOWER

Sets the transmit power of the DW1000.

```shell title="Usage"
AT+TXPOWER
AT+TXPOWER <mode>
AT+TXPOWER <stage> <coarse> <fine>
```

=== "Argument Descriptions"

    | Argument | Description                                |
    |:---------|:-------------------------------------------|
    | `mode`   | The preset power mode to use.              |
    | `stage`  | The amplification stage to adjust          |
    | `coarse` | The coarse gain of the amplification stage |
    | `fine`   | The fine gain of the amplication stage     |

=== "Mode Descriptions"

    | Value | Description          |
    |:------|:---------------------|
    | `0`   | The default TX power |
    | `1`   | The maximum TX power |

=== "Stage Descriptions"

    | Value | Description |
    |:------|:------------|
    | `0`   | BOOSTNORM   |
    | `1`   | BOOSTP500   |
    | `2`   | BOOSTP250   |
    | `3`   | BOOSTP125   |

=== "Coarse Descriptions"

    | Value | Description     |
    |:------|:----------------|
    | `0`   | Off (No output) |
    | `1`   | 0 dB            |
    | `2`   | 2.5 dB          |
    | `3`   | 5.0 dB          |
    | `4`   | 7.5 dB          |
    | `5`   | 10.0 dB         |
    | `6`   | 12.5 dB         |
    | `7`   | 15.0 dB         |

=== "Fine Descriptions"

    | Value | Description |
    |:------|:------------|
    | `0`   | 0.0 dB      |
    | `1`   | 0.5 dB      |
    | `2`   | 1.0 dB      |
    | `3`   | 1.5 dB      |
    | `4`   | 2.0 dB      |
    | `5`   | 2.5 dB      |
    | `6`   | 3.0 dB      |
    | `7`   | 3.5 dB      |
    | `8`   | 4.0 dB      |
    | `9`   | 4.5 dB      |
    | `10`  | 5.0 dB      |
    | `11`  | 5.5 dB      |
    | `12`  | 6.0 dB      |
    | `13`  | 6.5 dB      |
    | `14`  | 7.0 dB      |
    | `15`  | 7.5 dB      |
    | `16`  | 8.0 dB      |
    | `17`  | 8.5 dB      |
    | `18`  | 9.0 dB      |
    | `19`  | 9.5 dB      |
    | `20`  | 10.0 dB     |
    | `21`  | 10.5 dB     |
    | `22`  | 11.0 dB     |
    | `23`  | 11.5 dB     |
    | `24`  | 12.0 dB     |
    | `25`  | 12.5 dB     |
    | `26`  | 13.0 dB     |
    | `27`  | 13.5 dB     |
    | `28`  | 14.0 dB     |
    | `29`  | 14.5 dB     |
    | `30`  | 15.0 dB     |
    | `31`  | 15.5 dB     |

??? info "Calculating stage gain"

    The gain for a stage is calculated with the following formula:

    Gain = coarse~dB~ + fine~dB~

??? note

    The fine gain has no affect when the coarse gain is set to `0` (no output).
    However, this argument is still required if fine control of the amplifiers
    is desired.

???+ warning

    Increasing transmitter power supply can help UWB to maximum range, but the 
    maximum power supply exceed restricted transmit power level regulation.

### AT+STREAMMODE

Determines if the entire neighbor list is sent over serial or if only updates 
are sent over serial.

```shell title="Usage"
AT+STREAMMODE
AT+STREAMMODE <mode>
```

=== "Argument Descriptions"

    | Argument | Description               |
    |:---------|:--------------------------|
    | `mode`   | The neighbor display mode |

=== "Mode Descriptions"

    | Value | Description                             |
    |:------|:----------------------------------------|
    | `0`   | (Default) Entire neighbor list is sent. |
    | `1`   | Only neighbor updates are sent.         |

### AT+TWRMODE

Determines the UWB ranging protocol.

```shell title="Usage"
AT+TWRMODE
AT+TWRMODE <mode>
```

=== "Argument Descriptions"

    | Argument | Description                  |
    |:---------|:-----------------------------|
    | `mode`   | The ranging protocol to use. |

=== "Mode Descriptions"

    | Value | Description                                     |
    |:------|:------------------------------------------------|
    | `0`   | Single-sided two-way ranging (SS-TWR)           |
    | `1`   | (Default) Double-sided two-way ranging (DS-TWR) |

### AT+LEDMODE

Determines the LED display mode,

```shell title="Usage"
AT+LEDMODE
AT+LEDMODE <mode>
```

=== "Argument Descriptions"

    | Argument | Description   |
    |:---------|:--------------|
    | `mode`   | The LED mode. |

=== "Mode Descriptions"

    | Value | Description                                         |
    |:------|:----------------------------------------------------|
    | `0`   | (Default) LEDs are turned on and displaying states. |
    | `1`   | LEDs are turned off                                 |

### AT+PWRAMP
Determines the state of the external power amplifiers.

```shell title="Usage"
AT+PWRAMP
AT+PWRAMP <mode>
```

=== "Argument Descriptions"

    | Argument | Description                        |
    |:---------|:-----------------------------------|
    | `mode`   | The external power amplifier mode. |

=== "Mode Descriptions"

    | Value | Description                                         |
    |:------|:----------------------------------------------------|
    | `0`   | (Default) External power amplifiers are disabled.   |
    | `1`   | UWB amplifier enabled, BLE amplifier disabled.      |
    | `2`   | UWB amplifier disabled, BLE 10 dB gain.             |
    | `3`   | UWB amplifier enabled, BLE 10 dB gain.              |
    | `4`   | UWB amplifier disabled, BLE 20 dB gain.             |
    | `5`   | UWB amplifier enabled, BLE 20 dB gain.              |

???+ info

    This requires the BELUGA_RANGE_EXTENSION (todo: insert link) build-time 
    configuration to be enabled.

### AT+FORMAT

Sets the format mode for serial outputs.

```shell title="Usage"
AT+FORMAT
AT+FORMAT <mode>
```

=== "Argument Descriptions"

    | Argument | Description                   |
    |:---------|:------------------------------|
    | `mode`   | The output format mode to use |

=== "Mode Descriptions"

    | Value | Description          |
    |:------|:---------------------|
    | `0`   | (Default) CSV format |
    | `1`   | JSON format          |
    | `2`   | Framed format        |

??? info "Modes"

    **CSV format** is the most basic mode. It prints the command outputs
    normally and the neighbor values are printed in a comma-separated
    fashion with line-endings separating neighbor entries. Neighbor
    removals are not recorded.

    **JSON format** is very similar to CSV format in its behavior. The
    main differences are that the neighbors are sent over serial in JSON format
    instead of CSV format, and neighbor removals are noted with `rm <id>`.

    **Framed format** places all serial output into frames. See 
    (todo: insert link here) for more information on Beluga frames.

### AT+PHR
Determines whether extended frame lengths are allowed.

```shell title="Usage"
AT+PHR
AT+PHR <mode>
```

=== "Argument Descriptions"

    | Argument | Description                           |
    |:---------|:--------------------------------------|
    | `mode`   | Allow transmit frame length extension |

=== "Mode Descriptions"

    | Value | Description                          |
    |:------|:-------------------------------------|
    | `0`   | (Default) Use standard frame lengths |
    | `1`   | Allow frame lengths up to 1023 bytes |

???+ warning

    Updating this parameter also has an affect on the BLE advertising data. Nodes with mismatching
    data will not attempt to range to each other.

### AT+DATARATE
Determines the data rate for the DW1000.

```shell title="Usage"
AT+DATARATE
AT+DATARATE <rate>
```

=== "Argument Descriptions"

    | Argument | Description                           |
    |:---------|:--------------------------------------|
    | `rate`   | The data rate of the DW1000.          |

=== "Rate Descriptions"

    | Value | Description          |
    |:------|:---------------------|
    | `0`   | (Default) 6.8Mbits/s |
    | `1`   | 850 kbit/s           |
    | `2`   | 110 kbit/s           |

???+ warning

    Updating this parameter also has an affect on the BLE advertising data. Nodes with mismatching
    data will not attempt to range to each other.

### AT+PULSERATE
Determines the pulse repetition frequency of the DW1000.

```shell title="Usage"
AT+PULSERATE
AT+PULSERATE <rate>
```

=== "Argument Descriptions"

    | Argument | Description                            |
    |:---------|:---------------------------------------|
    | `rate`   | The pulse repetition frequency to use. |

=== "Rate Descriptions"

    | Value | Description      |
    |:------|:-----------------|
    | `0`   | 16 MHz           |
    | `1`   | (Default) 64 MHz |

???+ warning

    Updating this parameter also has an affect on the BLE advertising data. Nodes with mismatching
    data will not attempt to range to each other.

### AT+PREAMBLE
Determines the preamble length of the DW1000.

```shell title="Usage"
AT+PREAMBLE
AT+PREAMBLE <preamble>
```

=== "Argument Descriptions"

    | Argument   | Description                        |
    |:-----------|:-----------------------------------|
    | `preamble` | The preamble length of the DW1000. |

=== "Preamble Descriptions"

    | Value  | Description                            |
    |:-------|:---------------------------------------|
    | `64`   | 64 symbols per transmission            |
    | `128`  | (Default) 128 symbols per transmission |
    | `256`  | 256 symbols per transmission           |
    | `512`  | 512 symbols per transmission           |
    | `1024` | 1024 symbols per transmission          |
    | `1536` | 1536 symbols per transmission          |
    | `2048` | 2048 symbols per transmission          |
    | `4096` | 4096 symbols per transmission          |

???+ warning

    Updating this parameter also has an affect on the BLE advertising data. Nodes with mismatching
    data will not attempt to range to each other.

### AT+PAC
Determines the preamble acquisition chunk size for the DW1000.

```shell title="Usage"
AT+PAC
AT+PAC <size>
```

=== "Argument Descriptions"

    | Argument   | Description                               |
    |:-----------|:------------------------------------------|
    | `size`     | he size of the preamble acquisition chunk |

=== "Size Descriptions"

    | Value | Description                                                             |
    |:------|:------------------------------------------------------------------------|
    | `0`   | (Default) 8 bytes (recommended for RX of preamble length 128 and below) |
    | `1`   | 16 bytes (recommended for RX of preamble length 256)                    |
    | `2`   | 32 bytes (recommended for RX of preamble length 512)                    |
    | `3`   | 64 bytes (recommended for RX of preamble length 1024 and above)         |

???+ warning

    Updating this parameter also has an affect on the BLE advertising data. Nodes with mismatching
    data will not attempt to range to each other.

### AT+SFD
Determines the length of the start of frame delimiter on the DW1000.

```shell title="Usage"
AT+SFD
AT+SFD <mode>
```

=== "Argument Descriptions"

    | Argument | Description          |
    |:---------|:---------------------|
    | `mode`   | The SFD mode to use. |

=== "Mode Descriptions"

    | Value | Description                                                    |
    |:------|:---------------------------------------------------------------|
    | `0`   | (Default) Standard SFD as defined in the IEEE802.15.4 standard |
    | `1`   | DecaWave Proprietary SFD                                       |

???+ warning

    Updating this parameter also has an affect on the BLE advertising data. Nodes with mismatching
    data will not attempt to range to each other.

### AT+PANID
Determines the Personal Area Network (PAN) ID for the DW1000. This allows for different networks to be in the same area.

```shell title="Usage"
AT+PANID
AT+PANID <id>
```

| Argument | Description                                                                      |
|:---------|:---------------------------------------------------------------------------------|
| `id`     | The id of the personal area network. Allowable range: [0,65535]. Default: 57034. |

???+ warning

    Updating this parameter also has an affect on the BLE advertising data. Nodes with mismatching
    data will not attempt to range to each other.

### AT+EVICT
Determines the eviction policy of the neighbor list.

```shell title="Usage"
AT+EVICT
AT+EVICT <scheme>
```

=== "Argument Descriptions"

    | Argument | Description                                                 |
    |:---------|:------------------------------------------------------------|
    | `scheme` | The eviction scheme to use when managing the neighbor list. |

=== "Scheme Descriptions"

    | Value | Description                                                                                                                                                                                                                                                                                                            |
    |:------|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
    | `0`   | Index RR. Use an index round-robin strategy. This will evict node 0 first, then node 1 on the next insertion, then node 2 on the following insertion. After evicting N - 1 nodes (N being the maximum number of nodes the neighbor list can hold), then node 0 will be evicted next. This strategy is not recommended. |
    | `1`   | (Default) RSSI, Evicts the node with the lowest RSSI. If there are no nodes with a lower RSSI than the scanned node, then no neighbors are evicted.                                                                                                                                                                    |
    | `2`   | Range. Evict the node with the largest ranging value.                                                                                                                                                                                                                                                                  |
    | `3`   | LRS. Evict the node that has not been scanned by the BLE in the longest amount of time.                                                                                                                                                                                                                                |
    | `4`   | LRR. Evict the node that has been successfully ranged to in the longest amount of time.                                                                                                                                                                                                                                |

???+ info

    This requires the BELUGA_EVICT_RUNTIME_SELECT (todo: insert link) build-time configuration to be enabled.

### AT+VERBOSE
Sets the verbosity of the command responses.

```shell title="Usage"
AT+VERBOSE
AT+VERBOSE <mode>
```

=== "Argument Descriptions"

    | Argument | Description              |
    |:---------|:-------------------------|
    | `mode`   | The verbose mode setting |

=== "Mode Descriptions"

    | Value | Description                       |
    |:------|:----------------------------------|
    | `0`   | (Default) Verbose mode turned off |
    | `1`   | Verbose mode turned on            |

### AT+CALIBRATE
Calibrate the UWB antenna delays.

```shell title="Usage"
AT+CALIBRATE
AT+CALIBRATE <delay id> <calibration value>
```

=== "Argument Descriptions"

    | Argument            | Description                                           |
    |:--------------------|:------------------------------------------------------|
    | `delay id`          | The value to calibrate the antenna delay for.         |
    | `calibration value` | The calibration value for the specified antenna delay |

=== "Delay ID Descriptions"

    | Value | Description                                                    |
    |:------|:---------------------------------------------------------------|
    | `0`   | The delay ID for the UWB receive path with a PRF[^2] of 16MHz  |
    | `1`   | The delay ID for the UWB receive path with a PRF[^2] of 64MHz  |
    | `2`   | The delay ID for the UWB transmit path with a PRF[^2] of 16MHz |
    | `3`   | The delay ID for the UWB transmit path with a PRF[^2] of 64MHz |

    [^2]: The PRF is set with the [AT+PULSERATE](commands.md#atpulserate) command. (todo: link command)

### AT+WAITUSBHOST
Set the USB device start-up mode.

```shell title="Usage"
AT+WAITUSBHOST
AT+WAITUSBHOST <mode>
```

=== "Argument Descriptions"

    | Argument | Description                  |
    |:---------|:-----------------------------|
    | `mode`   | The USB device start-up mode |

=== "Mode Descriptions"

    | Value | Description                                   |
    |:------|:----------------------------------------------|
    | `0`   | (Default) Do not wait for USB host connection |
    | `1`   | Wait for USB host connection                  |

???+ info
    
    This requires the USB_DEVICE_STACK (todo: insert link) build-time configuration to be enabled.

## Control Commands

### AT+STARTUWB
Starts UWB initiator and responder.

```shell title="Usage"
AT+STARTUWB
```

### AT+STOPUWB
Stops UWB initiator and responder.

```shell title="Usage"
AT+STOPUWB
```

### AT+STARTBLE
Starts BLE advertising and scanning.

```shell title="Usage"
AT+STARTBLE
```

### AT+STOPBLE
Stops BLE advertising and scanning.

```shell title="Usage"
AT+STOPBLE
```

### AT+REBOOT
Reboots the Beluga node. This will also cause the USB connection to be lost.

```shell title="Usage"
AT+REBOOT
```

### AT+ANTENNA
Selects the antenna for BLE usage.

```shell title="Usage"
AT+ANTENNA <antenna>
```

=== "Argument descriptions"

    | Argument  | Description                |
    |:----------|:---------------------------|
    | `antenna` | The antenna to use for BLE |

=== "Antenna Descriptions"

    | Value | Description       |
    |:------|:------------------|
    | `1`   | Primary antenna   |
    | `2`   | Secondary antenna |

??? info "Reading antenna value"

    See AT+STATUS

???+ info

    This requires the BELUGA_RANGE_EXTENSION (todo: insert link) build-time 
    configuration to be enabled.

### AT+DEEPSLEEP
Places Beluga into deep sleep. A wakeup source must be configured for Beluga to wake up (there are none currently).

```shell title="Usage"
AT+DEEPSLEEP
```

### AT+SYNC
Synchronize the wireless settings of a certain node on the network.

```shell title="Usage"
AT+SYNC <id>
```

| Argument | Description                                      |
|:---------|:-------------------------------------------------|
| `id`     | The ID of the node on the network to synchronize |

### AT+STARVE
Starves a watchdog timer channel for testing purposes.

```shell title="Usage"
AT+STARVE <channel>
```

=== "Argument Descriptions"

    | Argument  | Description                    |
    |:----------|:-------------------------------|
    | `channel` | The watchdog channel to starve |

=== "Channel Descriptions"

    | Value | Description                   |
    |:------|:------------------------------|
    | `0`   | Ranging initiator channel     |
    | `1`   | Responder channel             |
    | `2`   | Neighbor list monitor channel |
    | `3`   | Commands channel              |

### AT+EXCHANGE
Set the exchange counter to a new value

```shell title="Usage"
AT+EXCHANGE <value>
```

| Argument | Description                                      |
|:---------|:-------------------------------------------------|
| `value`  | The new exchange value to start counting up from |

## Status Commands

### AT+TIME
Retrieve the current Beluga timestamp (ms since boot).

```shell title="Usage"
AT+TIME
```

### AT+STATUS
Retrieve the firmware information and the current states.

```shell title="Usage"
AT+STATUS
```

=== "Status Return Fields"

    | Bitfield | Name       | Description                                                                             |
    |:--------:|:-----------|:----------------------------------------------------------------------------------------|
    |  31:12   | -          | Reserved                                                                                |
    |    11    | EVICT_ALGO | 0: eviction algorithm is fixed to the build<br>1: eviction algorithm runtime selectable |
    |    10    | ANT_SEL    | 0: Primary antenna used<br>1: Secondary antenna used                                    |
    |    9     | UWB        | 0: UWB inactive<br>1: UWB active                                                        |
    |    8     | BLE        | 0: BLE inactive<br>1: BLE active                                                        |
    |   7:0    | BOARD      | The hardware platform ID                                                                |

=== "Hardware Platform IDs"

    | Value | Board       |
    |:------|:------------|
    | 0     | DWM1001_dev |
    | 1     | Beluga      |
    | >=2   | Unspecified |

### AT+VERSION
Retrieve the firmware version.

```shell title="Usage"
AT+VERSION
```

### AT+REASON
Retrieve the reboot reason.

```shell title="Usage"
AT+REASON
```

| Reason                                                     | Value[^3] |
|:-----------------------------------------------------------|:----------|
| External pin                                               | 1         |
| Software reset                                             | 2         |
| Brownout (drop in voltage)                                 | 4         |
| Power-on reset (POR)                                       | 8         |
| Watchdog timer expiration                                  | 16        |
| Debug event                                                | 32        |
| Security Violation                                         | 64        |
| Waking up from low power mode                              | 128       |
| CPU lock-up detected                                       | 256       |
| Parity error                                               | 512       |
| PLL error                                                  | 1024      |
| Clock error                                                | 2048      |
| Hardware reset                                             | 4096      |
| User reset                                                 | 8192      |
| Temperature reset                                          | 16384     |
| Bootloader reset (entry/exit)                              | 32768     |
| Flash ECC reset                                            | 65536     |

[^3]: The values listed can be OR’ed together to give multiple reset reasons.

???+ info

    This requires the BELUGA_RESET_REASON (todo: insert link) build-time configuration to be enabled.

### AT+NEIGHBORS
Print the scanned neighbor nodes in the neighbor list.

```shell title="Usage"
AT+NEIGHBORS
```
