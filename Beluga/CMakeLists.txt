#
# Copyright (c) 2021 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Beluga)

# NORDIC SDK APP START
file(GLOB SOURCE "src/*.c")
file(GLOB DECAWAVE_SRC "DecaDriver/src/*.c")

include_directories(include DecaDriver/include)

target_sources(app PRIVATE
  ${SOURCE}
  ${DECAWAVE_SRC}
)
# NORDIC SDK APP END
