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

Appendix
========
Adding Board Roots
------------------
In order for Zephyr to find Beluga, you need to specify a Board Root. In VS Code, this is
done by navigating to File->Preferences->Settings or by just pressing :kbd:`CTRL+,`.
Then under **Extensions**, find **nRF Connect** navigate to **Board Roots**. Add the absolute
path to the **Beluga-2.0** repository to the board roots.
If you are using the command line, run `make beluga`
