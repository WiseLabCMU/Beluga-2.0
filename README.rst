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
    │   ├── CMakeLists.txt
    │   ├── config
    │   ├── DecaDriver
    │   │   ├── include
    │   │   └── src
    │   ├── dts
    │   │   └── bindings
    │   ├── images
    │   │   └── decawave_dwm1001_dev
    │   ├── include
    │   ├── Kconfig
    │   ├── Kconfig.sysbuild
    │   ├── Makefile
    │   ├── overlay
    │   ├── prj.conf
    │   ├── src
    │   └── VERSION
    ├── boards
    │   └── arm
    │       └── Beluga
    ├── DW1000_docs
    ├── JLink
    │   └── JLink_Linux_V798a_x86_64.deb
    ├── Makefile
    ├── README.rst
    ├── ROS
    │   └── src
    │       ├── beluga
    │       ├── beluga_messages
    │       └── example_beluga_client
    └── setup.sh

Setup and Building
==================
It is recommended to use the nRF Connect extension in Visual Studio Code to build.

Setup (VS code)
---------------
1. Download and install `nRF Connect for Desktop`_
2. Download and install `VS Code`_
3. Open nRF Connect
4. Install the toolchain manager
5. Open the toolchain manager
6. Install nRF Connect SDK v2.7.0
7. Open VS code and install the nRF Connect extension pack
8. Open Beluga-2.0/Beluga in VS Code

For additional help, refer to `Nordic's nRF Connect installation guide`_

.. _nRF Connect for Desktop: https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop
.. _VS Code: https://code.visualstudio.com/download'
.. _Nordic's nRF Connect installation guide: https://docs.nordicsemi.com/bundle/nrf-connect-desktop/page/index.html

Setup (Command line, Linux Only)
--------------------
Run the following command:

.. code-block:: bash

    ./setup.sh

Build Configurations
--------------------
Before building your application, you need to set up your build configurations. There are two important build
configurations: Beluga and decawave_dwm1001_dev. The build configurations are listed below.

decawave_dwm1000_dev
^^^^^^^^^^^^^^^^^^^^
* **Board Target:** decawave_dwm1001_dev
* **Base configuration file:** prj.conf
* **Base device tree overlay:** overlay/decawave_dwm1001_dev.overlay
* **Optimization level:** Os (Optimize for size)
* **Sysbuild:** No sysbuild

Beluga
^^^^^^
See `Adding Board Roots <#adding-board-roots>`_ for finding custom boards.

* **Board Target:** Beluga
* **Base configuration file:** prj.conf
* **Extra Kconfig fragments:** config/beluga.conf and config/usb.conf
* **Base Device tree overlay:** overlay/beluga.overlay
* **Optimization level:** Anything works
* **Sysbuild:** No sysbuild

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
    AT+TXPOWER

Determines the UWB transmitter power setting.
No argument will return the current UWB transmitter power setting.
This setting is saved in flash.

+-------------+------------------------+
| mode        | Description            |
+=============+========================+
| 0 (Default) | Default power supply   |
+-------------+------------------------+
| 1           | Maximum power supply   |
+-------------+------------------------+

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
|             | active                      |
+-------------+-----------------------------+

.. note::
    This command is not supported on the decawave_dwm1001m_dev board

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
    This command is not supported on the decawave_dwm1001m_dev board

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

Appendix
========
Adding Board Roots
------------------
In order for Zephyr to find Beluga, you need to specify a Board Root. In VS Code, this is
done by navigating to File->Preferences->Settings or by just pressing :kbd:`CTRL+,`.
Then under **Extensions**, find **nRF Connect** navigate to **Board Roots**. Add the absolute
path to the **Beluga-2.0** repository to the board roots.
If you are using the command line, run `make beluga`
