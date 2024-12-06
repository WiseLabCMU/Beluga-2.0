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

Determines the ID number of the number. No argument will return the current setting. This setting is saved in flash.

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

Determines how the node should behave when reset/powered on. No argument will return the current boot mode.

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

Appendix
========
Adding Board Roots
------------------
In order for Zephyr to find Beluga, you need to specify a Board Root. In VS Code, this is
done by navigating to File->Preferences->Settings or by just pressing :kbd:`CTRL+,`.
Then under **Extensions**, find **nRF Connect** navigate to **Board Roots**. Add the absolute
path to the **Beluga-2.0** repository to the board roots.
If you are using the command line, run `make beluga`
