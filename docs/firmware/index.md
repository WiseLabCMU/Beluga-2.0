## Overview
- Mention something about the old firmware
- Architecture

## Requirements
These are the following requirements to build and flash the Beluga firmware

- [nRF Command Line tools](https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools/Download){target="+blank"}
- [nRF Connect SDK](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/installation/install_ncs.html){target="+blank"}
    - v2.9.0
- [SEGGER JLink](https://www.segger.com/downloads/jlink){target="+blank"}

## Supported Hardware
The Beluga firmware can be built for the following hardware platforms:

- [DWM1001-dev](https://www.qorvo.com/products/p/DWM1001-DEV){target="_blank"}
- [Beluga](../hardware/index.md)

Additional hardware platforms may be added.

## Stuff
- Build configs
  - Custom configs
  - external flash configs
  - image signing
  - Device tree
- How to build and flash
  - Through code
  - CLI
  - mcuboot
    - AuTerm
    - in-tree CLI tool