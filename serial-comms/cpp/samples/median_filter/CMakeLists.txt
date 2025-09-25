cmake_minimum_required(VERSION 3.22)
project(median_filter)

set(CMAKE_CXX_STANDARD 20)

set(BELUGA_SERIAL_DIR ../../beluga-serial)

add_subdirectory(${BELUGA_SERIAL_DIR} beluga-serial-lib)

add_executable(filter_sample src/main.cpp)
target_link_libraries(filter_sample beluga-serial)
