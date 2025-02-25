==========
Beluga 2.0
==========

Decawave DWM1001-DEV board firmware for building ad-hoc distance measurement network

Uses BLE to discover and UWB to measure distance between nodes

Overview
========
Beluga makes it more convenient for users to create an ad-hoc
distance measurement network. Users can enter different types
of AT commands through serial monitor to configure the properties
of the DWM1001-DEV board. For example, change the channel of the
UWB signal and polling frequency. These features enable users to
customize their own network setup and meet the requirements. Beluga
can be applied in the area of localization, robotics, and
infrastructure sensing.

Project Structure
=================
.. code-block:: none

    Beluga-2.0
    ├── Beluga
    │   ├── boards
    │   │   └── wiselab
    │   │       └── beluga
    │   ├── config
    │   ├── DecaDriver
    │   ├── dts
    │   ├── images
    │   ├── include
    │   ├── overlay
    │   │   └── extra
    │   └── src
    ├── Documentation
    │   ├── Beluga
    │   └── DW1000_docs
    ├── Hardware
    ├── JLink
    ├── ROS
    │   └── src
    │       ├── beluga
    │       ├── beluga_messages
    │       └── example_beluga_client
    └── serial-comms
        ├── architectures
        ├── cpp
        │   └── beluga-serial
        └── python
            └── beluga_serial



Setup and Building
==================
It is recommended to use the nRF Connect extension in Visual Studio Code to build.

Option 1: Setup (VS code, All platforms)
---------------
1. Download and install `nRF Connect for Desktop`_
2. Download and install `VS Code`_
3. Download and install `nRF Command Line Tools`_
4. Open nRF Connect
5. Install the toolchain manager
6. Open the toolchain manager
7. Install nRF Connect SDK v2.9.0
8. Open VS code and install the nRF Connect extension pack
9. Open Beluga-2.0/Beluga in VS Code

For additional help, refer to `Nordic's nRF Connect installation guide`_

.. _nRF Connect for Desktop: https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop
.. _VS Code: https://code.visualstudio.com/download
.. _nRF Command Line Tools: https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools/Download?lang=en#infotabs
.. _Nordic's nRF Connect installation guide: https://docs.nordicsemi.com/bundle/nrf-connect-desktop/page/index.html

Option 2: Setup (Command line, Linux Only, Not recommended)
--------------------
Run the following command:

.. code-block:: bash

    ./setup.sh

Set up for ROS node dependencies (Tested on ROS Humble)
-------------------------------------------------------
1. Install `ROS Humble`_

.. _ROS Humble: https://docs.ros.org/en/humble/Installation.html

ROS node
--------
This project contains a ROS node that can communicate with a Beluga node. To build and run it, navigate to the
``ROS`` directory and follow the `ROS package installation and usage`_ instructions. The node contains 3 publishers
and 1 service:

1. Publisher for neighborhood list updates (additions or removals of neighbors)
2. Publisher for distances to neighbors (Only updates)
3. Publisher for responding to neighbor distancing requests
4. Service for the Beluga AT commands

By default, these publishers and service are named ``neighbor_list``, ``range_updates``, ``range_exchanges``, and
``at_command``, respectively. These topic names can be customized through the ROS args.

ROS Arguments
^^^^^^^^^^^^^
+--------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| Argument           | Description                                                                                                                                                                                |
+--------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``neighbors_name`` | Topic name for the neighborhood list updates publisher                                                                                                                                     |
+--------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``ranges_name``    | Topic name for the range updates publisher                                                                                                                                                 |
+--------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``exchange_name``  | Topic name for the ranging exchanges publisher                                                                                                                                             |
+--------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``service_topic``  | Topic name for the AT command service                                                                                                                                                      |
+--------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``history_depth``  | The publisher queue depth for all the publishers in the node                                                                                                                               |
+--------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``port``           | The specific port to connect to. Note: This may change mid program due to the node rebooting. The node ID is fetched and saved during execution to ensure the same node is reconnected to. |
+--------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``config``         | JSON file for custom node configurations                                                                                                                                                   |
+--------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+

Example Usage

.. code-block:: bash

    ros2 run beluga beluga --ros-args --param port:=/dev/ttyACM1

Starter JSON file for customized settings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: json

    {
        "id": 1,
        "bootmode": 2,
        "rate": 100,
        "channel": 5,
        "timeout": 9000,
        "txpower": 1,
        "streammode": 1,
        "twrmode": 1,
        "ledmode": 0,
        "pwramp": 1,
        "antenna": 1,
        "phr": 0,
        "datarate": 0,
        "pulserate": 0,
        "preamble": 128,
        "pac": 0,
        "sfd": 0,
        "panid": 41760
    }

The above JSON can be used as a starting file. If a default setting is desired,
delete the entry from the JSON.


.. _ROS package installation and usage: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#build-a-package

Build Configurations
--------------------
Before building your application, you need to set up your build configurations. There are two important build
configurations: Beluga and `decawave_dwm1001_dev`_. The build configurations are listed below. To create these build
configurations, open the nRF Connect Extension in VS code by pressing the icon or using the shortcut :kbd:`CTRL+ALT+N`

decawave_dwm1000_dev
^^^^^^^^^^^^^^^^^^^^
* **Board Target:** decawave_dwm1001_dev
* **Base configuration file:** prj.conf
* **Base device tree overlay:** overlay/decawave_dwm1001_dev.overlay
* **Optimization level:** Os (Optimize for size)
* **Sysbuild:** No sysbuild

.. note::

    This board can theoretically be built with sysbuild, however, due to memory constraints, it will fail
    because the program memory is split into 2 sections, each section being 200 kB of flash. Since the current
    firmware requires more than 200 kB of flash, sysbuild with McuMgr is not used. Additionally, McuMgr is not
    necessary since this board comes with a built in J-Link debuggger.

Beluga
^^^^^^
See `Adding Board Roots <#adding-board-roots>`_ for finding custom boards.

* **Board Target:** Beluga
* **Base configuration file:** prj.conf
* **Extra Kconfig fragments:** config/beluga.conf, config/usb.conf, and config/mcumgr.conf
* **Base Device tree overlay:** overlay/beluga.overlay
* **Extra Device tree overlays:** overlay/extra/usb.overlay
* **Optimization level:** Anything works
* **Sysbuild:** Use sysbuild

.. note::

    If a larger program section is desired for Beluga, then that leaves 2 options. The first option is to compile
    without McuMgr (Exclude config/mcumgr.conf and overlay/extra/usb.overlay and build with the No sysbuild flag), or
    to build the hardware with the external flash.

Building and Flashing
---------------------
1. Select the nRF Connect Icon in the side bar
2. Select the build configuration you want to build.
3. Press the `build` button under **Actions**. If a clean build is desired, press the redo icon when hovering over build (pristine build)
4. Press the `flash` button under **Actions**. If multiple targets are connected, select the desired target from the dropdown list.

AT Commands
===========
The following AT commands can help users to access and modify DWM1001-DEV firmware to meet specific need.
There are a total of 26 commands, and certain configurations will be saved in flash memory to restore user
settings after the system reboots.

Commands:

1. `ID <#id>`_
2. `STARTBLE <#startble>`_
3. `STOPBLE <#stopble>`_
4. `STARTUWB <#startuwb>`_
5. `STOPUWB <#stopuwb>`_
6. `BOOTMODE <#bootmode>`_
7. `RATE <#rate>`_
8. `CHANNEL <#channel>`_
9. `RESET <#reset>`_
10. `TIMEOUT <#timeout>`_
11. `TXPOWER <#txpower>`_
12. `STREAMMODE <#streammode>`_
13. `TWRMODE <#twrmode>`_
14. `LEDMODE <#ledmode>`_
15. `REBOOT <#reboot>`_
16. `PWRAMP <#pwramp>`_
17. `ANTENNA <#antenna>`_
18. `TIME <#time>`_
19. `FORMAT <#format>`_
20. `DEEPSLEEP <#deepsleep>`_
21. `PHR <#phr>`_
22. `DATARATE <#datarate>`_
23. `PULSERATE <#pulserate>`_
24. `PREAMBLE <#preamble>`_
25. `PAC <#pac>`_
26. `SFD <#sfd>`_

ID
--
.. code-block:: none

    AT+ID <number>
    AT+ID

Determines the ID number of the number.
No argument will return the current setting.
This setting is saved in flash.

.. note::

    <number> should be a positive, non-zero integer, and each node should have a unique ID.

STARTBLE
--------
.. code-block:: none

    AT+STARTBLE

Starts BLE broadcating/retrieving.

STOPBLE
-------
.. code-block:: none

    AT+STOPBLE

Stops BLE broadcating/retrieving.

STARTUWB
--------
.. code-block:: none

    AT+STARTUWB

Starts UWB initiator/responder.

STOPUWB
-------
.. code-block:: none

    AT+STOPUWB

Stops UWB initiator/responder.

BOOTMODE
--------
.. code-block:: none

    AT+BOOTMODE <mode>
    AT+BOOTMODE

Determines how the node should behave when reset/powered on.
No argument will return the current boot mode.
This setting is saved in flash.

+-------------+------------------------+
| mode        | Description            |
+=============+========================+
| 0 (Default) | Do nothing on startup  |
|             | (BLE and UWB off)      |
+-------------+------------------------+
| 1           | Start BLE              |
|             | broadcasting/receiving |
|             | on startup             |
+-------------+------------------------+
| 2           | Start BLE and UWB on   |
|             | startup, full          |
|             | functionality.         |
+-------------+------------------------+

.. note::
    For BOOTMODEs 1 and 2, the AT+ID command must have been previously ran, the last set ID will be used on startup.

RATE
----
.. code-block:: none

    AT+RATE <period>
    AT+RATE

Determines the frequency that the node send poll messages.
No argument will return the current polling period.
This setting is saved in flash.

+-----------+-------+-------+---------+
| Parameter | Input | Units | Default |
+-----------+-------+-------+---------+
| period    | 0-500 | ms    | 250     |
+-----------+-------+-------+---------+

.. note::
    When the frequency is 0, the node is in listening mode (It only responds to ranging requests)

CHANNEL
-------
.. code-block:: none

    AT+CHANNEL <channel>
    AT+CHANNEL

Determines the UWB signal's channel.
No argument will return the current UWB channel.
This setting is saved in flash.

+-----------+---------------+---------+
| Parameter | Valid Options | Default |
+-----------+---------------+---------+
| channel   | 1, 2, 3, 4,   | 5       |
|           | 5, 7          |         |
+-----------+---------------+---------+

.. note::
    The corresponding centre frequency and bandwidth of each channel please reference DW1000 User Manual (Section 10.5)

TXPOWER
-------
.. code-block:: none

    AT+TXPOWER <mode>
    AT+TXPOWER <stage> <coarse gain> <fine gain>
    AT+TXPOWER

Determines the UWB transmitter power setting.
No argument will return the current UWB transmitter power setting.
This setting is saved in flash.

One argument will set the power level to either the default power level or maximum power level.

+-------------+------------------------+
| mode        | Description            |
+=============+========================+
| 0 (Default) | Default power supply   |
+-------------+------------------------+
| 1           | Maximum power supply   |
+-------------+------------------------+

Three arguments allow for total control over the power setting. For example, if coarse gain is 2 and fine gain is 2, then the TX power will be 2.5 dB + 1.0 dB = 3.5 dB.

+-------------+-------+-------------------+
| Parameter   | Value | Description       |
+=============+=======+===================+
|             | 0     | BOOSTNORM         |
|             +-------+-------------------+
|             | 1     | BOOSTP500         |
|    stage    +-------+-------------------+
|             | 2     | BOOSTP250         |
|             +-------+-------------------+
|             | 3     | BOOSTP125         |
+-------------+-------+-------------------+
|             | 0     | Off (No output)   |
|             +-------+-------------------+
|             | 1     | 0 dB Gain         |
|             +-------+-------------------+
| coarse gain | 2     | 2.5 dB Gain       |
|             +-------+-------------------+
|             | ⋮     | 2.5 dB Gain Steps |
|             +-------+-------------------+
|             | 7     | 15 dB Gain        |
+-------------+-------+-------------------+
|             | 0     | 0.0 dB Gain       |
|             +-------+-------------------+
|             | 1     | 0.5 dB Gain       |
|             +-------+-------------------+
|  fine gain  | 2     | 1.0 dB Gain       |
|             +-------+-------------------+
|             | ⋮     | 0.5 dB Gain Steps |
|             +-------+-------------------+
|             | 31    | 15.5 dB gain      |
+-------------+-------+-------------------+


.. note::
    Increasing transmitter power supply can help UWB to maximum range, but the maximum power supply exceeds
    restricted transmit power level regulation.

TIMEOUT
-------
.. code-block:: none

    AT+TIMEOUT <elapsed time>
    AT+TIMEOUT

Determines the timeout parameter to evict nearby nodes.
No argument will return the current timeout setting.
This setting is saved in flash.

+-----------+--------+-------+---------+
| Parameter | Input  | Units | Default |
+-----------+--------+-------+---------+
| period    | 0-9000 | ms    | 9000    |
+-----------+--------+-------+---------+

STREAMMODE
----------
.. code-block:: none

    AT+STREAMMODE <mode>
    AT+STREAMMODE

Determines the neighbors list display mode.
No argument will return the current stream mode.
This setting is saved in flash.

+-------------+------------------------+
| mode        | Description            |
+=============+========================+
| 0 (Default) | Displays all           |
|             | neighbors, even those  |
|             | who have not been      |
|             | updated                |
+-------------+------------------------+
| 1           | Only display neighbors |
|             | that have been updated |
+-------------+------------------------+

TWRMODE
-------
.. code-block:: none

    AT+TWRMODE <mode>
    AT+TWRMODE

Determines the UWB ranging scheme.
No argument will return the current ranging scheme.
This setting is saved in flash.

+-------------+------------------------+
| mode        | Description            |
+=============+========================+
| 0           | Single-sided ranging   |
|             | (SS-TWR)               |
+-------------+------------------------+
| 1 (Default) | Double-sided ranging   |
|             | (DS-TWR)               |
+-------------+------------------------+

.. note::
    DS-TWR is more accurate and can reduce clock drift effect.
    SS-TWR can be used for a network that needs faster transmission.

LEDMODE
-------
.. code-block::
    AT+LEDMODE <mode>
    AT+LEDMODE

Determines the LED display mode.
No argument will return the current LED mode.
This setting is saved in flash.

+-------------+-----------------------------+
| mode        | Description                 |
+=============+=============================+
| 0 (Default) | LED support mode (All LEDs) |
+-------------+-----------------------------+
| 1           | No LEDSs support mode (turn |
|             | off all LEDs)               |
+-------------+-----------------------------+

.. note::
    LEDs support mode can be used for debugging, and another mode can be used for saving power.

RESET
-----
.. code-block::

    AT+RESET

Clear flash memory configuration. This command will reset all user configuration.

REBOOT
------
.. code-block::

    AT+REBOOT

Reboots Beluga. All internal states will be reset.

PWRAMP
------
.. code-block::

    AT+PWRAMP <mode>
    AT+PWRMAP

Determines if the BLE and UWB signals are amplified.
No argument will return the current amplifier setting.
This setting is saved in flash.

+-------------+-----------------------------+
| mode        | Description                 |
+=============+=============================+
| 0 (Default) | External amplifiers are     |
|             | inactive                    |
+-------------+-----------------------------+
| 1           | External amplifiers are     |
|             | active. The BLE amplifier   |
|             | is amplifying by 10 dB      |
+-------------+-----------------------------+
| 2           | External amplifiers are     |
|             | active. The BLE amplifier   |
|             | is amplifying by 22 dB      |
+-------------+-----------------------------+

.. note::
    This command is not supported on the `decawave_dwm1001_dev`_ board

ANTENNA
-------
.. code-block::

    AT+ANTENNA <antenna>
    AT+ANTENNA

Determines which antenna is used for neighbor discovery.
No argument will return the current antenna setting

+-----------+---------------+---------+
| Parameter | Valid Options | Default |
+-----------+---------------+---------+
| antenna   | 1, 2          | 1       |
+-----------+---------------+---------+

.. note::
    This command is not supported on the `decawave_dwm1001_dev`_ board

.. warning::
    This setting is not saved in flash

TIME
----
.. code-block::

    AT+TIME

Retrieves the current Beluga timestamp (ms since boot).

FORMAT
------
.. code-block::

    AT+FORMAT <mode>
    AT+FORMAT

Determines the formatting of the neighborhood list.
No argument will return the current format setting.
This setting is saved in flash.

+-------------+-----------------------------+
| mode        | Description                 |
+=============+=============================+
| 0 (Default) | CSV Format                  |
+-------------+-----------------------------+
| 1           | JSON Format                 |
|             | Removed neighbors are       |
|             | indicated by ``rm "ID"``    |
+-------------+-----------------------------+

DEEPSLEEP
---------
.. code-block::

    AT+DEEPSLEEP

Places Beluga into deep sleep, only allowing for a movement to wake Beluga.

PHR
---
.. code-block::

    AT+PHR <mode>
    AT+PHR

Determines the PHR mode used for UWB.
No argument will return the current PHR mode.
This setting is saved in flash.

+-------------+-----------------------------+
| mode        | Description                 |
+=============+=============================+
| 0 (Default) | Standard PHR Mode           |
+-------------+-----------------------------+
| 1           | DW proprietary extended     |
|             | frames PHR mode             |
+-------------+-----------------------------+

.. note::
    Refer to the DW1000 documents on how to best use this parameter

DATARATE
--------
.. code-block::

    AT+DATARATE <data rate>
    AT+DATARATE

Determines the data rate of the DW1000.
No argument will return the current data rate.
This setting is saved in flash.

+-------------+-----------------------------+
| data rate   | Description                 |
+=============+=============================+
| 0 (Default) | 6.8 MHz                     |
+-------------+-----------------------------+
| 1           | 850 kHz                     |
+-------------+-----------------------------+
| 2           | 110 kHz                     |
+-------------+-----------------------------+

.. note::
    Faster data rates mean faster transmission, but lower range. Refer to the DW1000 for appropriate use.

PULSERATE
---------
.. code-block::

    AT+PULSERATE <rate>
    AT+PULSERATE

Determines the pulse rate of the DW1000.
No arguments will return the current pulse rate.
This setting is saved in flash.

+-------------+-----------------------------+
| rate        | Description                 |
+=============+=============================+
| 0           | 64 Mhz                      |
+-------------+-----------------------------+
| 1 (Default) | 16 MHz                      |
+-------------+-----------------------------+

.. note::
    Refer to the DW1000 docs for appropriate use of this parameter.

PREAMBLE
--------
.. code-block::

    AT+PREAMBLE <preamble>
    AT+PREAMBLE

Determines the preamble length of the DW1000.
No arguments will return the current preamble length.
This setting is saved in flash.

+-----------+---------------+---------+
| Parameter | Valid Options | Default |
+-----------+---------------+---------+
| preamble  | 64, 128, 256, | 128     |
|           | 512, 1024,    |         |
|           | 1536, 2048,   |         |
|           | 4096          |         |
+-----------+---------------+---------+

.. note::
    A longer preamble length will increase range. Refer to the DW1000 docs for appropriate use.

PAC
---
.. code-block::

    AT+PAC <pac>
    AT+PAC

Determines the PAC size of the DW1000.
No arguments will return the current Preamble Acquisition Chunk (PAC) size.
This setting is saved in flash.

+-------------+-----------------------------+
| pac         | Description                 |
+=============+=============================+
| 0 (Default) | 8 bytes (recommended for RX |
|             | of preamble length 128 and  |
|             | below)                      |
+-------------+-----------------------------+
| 1           | 16 bytes (recommended for   |
|             | RX of preamble length 256)  |
+-------------+-----------------------------+
| 2           | 32 bytes (recommended for   |
|             | RX of preamble length 512)  |
+-------------+-----------------------------+
| 3           | 64 bytes (recommended for   |
|             | RX of preamble length 1024  |
|             | and up)                     |
+-------------+-----------------------------+

.. note::
    Refer to the DW1000 docs for more information

SFD
---
.. code-block::

    AT+SFD <mode>
    AT+SFD

Determines what SFD length to use for the DW1000.
No arguments will return the current SFD setting.
This setting is saved in flash.

+-------------+-----------------------------+
| mode        | Description                 |
+=============+=============================+
| 0 (Default) | Standard SFD length as      |
|             | defined in the IEEE802.15.4 |
|             | standard                    |
+-------------+-----------------------------+
| 1           | DW proprietary SFD (varies  |
|             | the length based on the     |
|             | data rate)                  |
+-------------+-----------------------------+

.. note::
    Refer to the DW1000 docs for more information

PANID
-----
.. code-block::

    AT+PANID <id>
    AT+PANID

Determines the Personal Area Network (PAN) ID for the DW1000.
No argument will return the current PAN ID setting.
This setting is saved in flash.

+-----------+---------+---------+
| Parameter | Input   | Default |
+-----------+---------+---------+
| id        | 0-65535 | 57034   |
+-----------+---------+---------+

Appendix
========
Adding Board Roots
------------------
In order for Zephyr to find Beluga, you need to specify a Board Root. In VS Code, this is
done by navigating to File->Preferences->Settings or by just pressing :kbd:`CTRL+,`.
Then under **Extensions**, find **nRF Connect** navigate to **Board Roots**. Add the absolute
path to the **Beluga-2.0** repository to the board roots.
If you are using the command line, run ``make beluga``

.. _decawave_dwm1001_dev: https://docs.zephyrproject.org/latest/boards/qorvo/decawave_dwm1001_dev/doc/index.html

Setup/Building/Flashing Troubleshooting
---------------------------------------

Unable to flash DW1001
^^^^^^^^^^^^^^^^^^^^^^
If you are seeing the following error "FATAL ERROR: one or more Python dependencies were missing; see the getting started guide for details on how to fix,"
it means that a python dependency is missing from the environment. This will require an update to the environment. Run the following steps to fix the
environment:

1. Open the toolchain manager, click on the dropdown arrow, and select "Generate environment script." Save the script to a location of your choice.
2. Open a terminal and source the environment script that you just generated (i.e source env.sh)
3. Find where the nordic toolchain is installed and navigate to it in the terminal (example path: ~/ncs/toolchains/2be090971e)
4. Run "./usr/local/bin/pip install pylink"
5. Open environment.json inside the toolchain directory, under the "LD_LIBRARY_PATH" key, add the following value to the list: "opt/nanopb/generator-bin/"

See `VS Code Extension - west flash fails from missing python dependencies`_ for more details.

.. _VS Code Extension - west flash fails from missing python dependencies: https://devzone.nordicsemi.com/f/nordic-q-a/100164/vs-code-extension---west-flash-fails-from-missing-python-dependencies/496078
