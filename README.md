# Beluga 2.0


*Decawave DWM1001-DEV board firmware for building ad-hoc distance measurement network*

*Uses BLE to discover and UWB to measure distance between nodes*

## Building
It is recommended to use the nRF Connect extension in Visual Studio Code to build.

### Setup (VS code)
1. Download and install [nRF Connect for Desktop](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop)
2. Download and install [VS Code](https://code.visualstudio.com/download)
3. Open nRF Connect
4. Install the Toolchain Manager
5. Open the toolchain manager
6. Install nRF Connect SDK v2.7.0
7. Open VS code and install the nRF Connect extension pack

#### Build Configurations
Before building your application, you need to set up your build configurations. There are two important build
configurations: Beluga and decawave_dwm1001_dev. The build configurations are listed below.

##### decawave_dwm1001_dev
* **Board Target:** decawave_dwm1001_dev/nrf52832
* **Base configuration file:** prj.conf
* **Base Device tree overlay:** overlay/decawave_dwm1001_dev.overlay
* **Optimization level:** Optimize for size
* **Sysbuild:** No sysbuild

##### Beluga
In order to find Beluga in the boards list, you need to add a new board root.
This can be done by navigating to File->Prefences->Settings (CTRL+Comma), then under **Extensions**, 
find **nRF Connect**. Navigate to **Board Roots** and add the absolute path to the Beluga-2.0 repo to the board roots. 

* **Board Target:** Beluga
* **Base configuration file:** prj.conf
* **Extra Kconfig fragments:** config/beluga.conf and config/usb.conf
* **Base Device tree overlay:** overlay/beluga.overlay
* **Optimization level:** Anything works
* **Sysbuild:** No sysbuild

## AT Command List
The following AT commands can help users to access and modify firmware to meet specific needs. There are total n commands
and commands 1, ..., n are stored in flash memory to set up user configuration after system reboot.

### 1. AT+ID
```
AT+ID <number>    Determines the ID number of the node
AT+ID             Retrieves the current ID of the node

NOTE: <number> should be a positive, non-zero integer, and each node should have a unique ID
```

### 2. AT+STARTBLE
```
AT+STARTBLE

Starts BLE broadcating/retrieving.
```

### 3. AT+STOPBLE
```
AT+STARTBLE

Stops BLE broadcating/retrieving.
```

### 4. AT+STARTUWB
```
AT+STARTUWB

Starts UWB initiator/responder.
```

### 5. AT+STOPUWB
```
AT+STOPUWB

Stops UWB initiator/responder.
```

### 6. AT+BOOTMODE
```
AT+BOOTMODE <mode>   Determines how the node should behave when reset/powered on.
AT+BOOTMODE          Gets the current behavior when powered on
<mode> = 0  -  Do nothing on startup (BLE and UWB off)
<mode> = 1  -  Start BLE broadcasting/receiving on startup
<mode> = 2  -  Start BLE and UWB on startup, full functionality.

Default setting: 0

NOTE: For BOOTMODEs 1 and 2, the AT+ID command must have been previously ran, the last set ID will be used on startup.
```

### 7. AT+RATE
```
AT+RATE <period>   Determines the frequency that the node send poll messages
AT+RATE            Gets the current period at which the node sends polling messages
<period> = 0-500 (units: ms)

Default setting: 250

NOTE: When the frequency is 0, the node is in listening mode (It only responds to ranging requests)
```

### 8. AT+CHANNEL
```
AT+CHANNEL <channel>   Determines the UWB signal's channel
AT+CHANNEL             Retrieves the current channel setting
Available <channel options: 1, 2, 3, 4, 5, 7

Default setting: 5

NOTE: The corresponding centre frequency and bandwidth of each channel please reference DW1000 User Manual (Section 10.5)
```

### 9. AT+TXPOWER
```
AT+TXPOWER <mode>   Determines the UWB transmitter power setting
AT+TXPOWER          Retrieves the current UWB transmitter power setting
<mode> = 0  -  Default power supply
<mode> = 1  -  Maximum power supply

Default setting: 0

NOTE: Increasing transmitter power supply can help UWB to maximum range, but the maximum power supply exceeds restricted transmit power level regulation.
```

### 10. AT+TIMEOUT
```
AT+TIMEOUT <elapsed time>   Determines the timeout parameter to evict nearby nodes
AT+TIMEOUT                  Retrieves the current timeout parameter to evict nodes

Default setting: 9000 (units: ms)

NOTE: This parameter indicates that if a nearby node does not update in <number> ms, the node will be evicted from another node's neighbor list.
```

### 11. AT+STREAMMODE
```
AT+STREAMMODE <mode>   Determines the UART display mode
AT+STREAMMODE          Retrieves the UART display mode

<mode> = 0  -  Whole neighbor list mode (Displays all neighbors, even when there are no updates)
<mode> = 1  -  Update ranges mode (Only displays nodes that have updated

Default setting: 0
```

### 12. AT+TWRMODE
```
AT+TWRMODE <mode>   Determines the UWB ranging scheme
AT+TWRMODE          Gets the current UWB ranging scheme
<mode> = 0  -  Single-sided ranging (SS-TWR)
<mode> = 1  -  Double-sided ranging (DS-TWR)

Default setting: 1

NOTE: DS-TWR is more accurate and can reduce clock drift effect. SS-TWR can be used for a network that needs faster transmission.
```

### 13. AT+LEDMODE
```
AT+LEDMODE <mode>   Determines the LED display mode
AT+LEDMODE          Retrieves the current LED display mode
<mode> = 0  -  LEDs support mode (turn on all LEDs)
<mode> = 1  -  No LEDSs support mode (turn off all LEDs)

Default setting: 0

NOTE: LEDs support mode can be used for debugging, and another mode can be used for saving power.
```

### 14. AT+RESET
```
AT+RESET   Clear flash memory configurations
This command will reset all user configurations, including
```
