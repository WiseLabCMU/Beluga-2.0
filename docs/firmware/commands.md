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
    data will not attempt to range to eachother.

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
