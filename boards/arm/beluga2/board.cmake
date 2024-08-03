# Copyright (c) 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=nrf52832_xxaa" "--speed=4000" "--reset-after-load")
board_runner_args(pyocd "--target=nrf52832")
set(OPENOCD_NRF5_SUBFAMILY "nrf52")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd-nrf5.board.cmake)
