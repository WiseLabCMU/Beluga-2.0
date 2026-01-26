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

## Status Commands
