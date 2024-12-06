==========
Beluga 2.0
==========

Decawave DWM1001-DEV board firmware for building ad-hoc distance measurement network

Uses BLE to discover and UWB to measure distance between nodes

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

.. dropdown:: Adding board roots
    :animate: fade-in
    :color: primary

    In order for Zephyr to find Beluga, you need to specify a Board Root. In VS Code, this is
    done by navigating to File->Preferences->Settings or by just pressing :kbd:`CTRL+,`.
    Then under **Extensions**, find **nRF Connect** navigate to **Board Roots**. Add the absolute
    path to the **Beluga-2.0** repository to the board roots.
    If you are using the command line, run `make beluga`

* **Board Target:** Beluga
* **Base configuration file:** prj.conf
* **Extra Kconfig fragments:** config/beluga.conf and config/usb.conf
* **Base Device tree overlay:** overlay/beluga.overlay
* **Optimization level:** Anything works
* **Sysbuild:** No sysbuild
