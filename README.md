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

