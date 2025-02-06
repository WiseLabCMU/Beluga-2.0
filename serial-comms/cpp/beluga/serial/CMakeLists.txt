cmake_minimum_required(VERSION 3.22)

add_library(serial)

file(GLOB SERIAL_C_API PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src/core/C-API/*.c)
file(GLOB SERIAL_SRC PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src/core/*.cpp)
file(GLOB SERIAL_TOOLS_CORE ${CMAKE_CURRENT_LIST_DIR}/src/tools/core/*.cpp)
file(GLOB SERIAL_TOOLS ${CMAKE_CURRENT_LIST_DIR}/src/tools/*.cpp)

target_sources(serial PRIVATE ${SERIAL_C_API} ${SERIAL_SRC} ${SERIAL_TOOLS_CORE} ${SERIAL_TOOLS})

target_include_directories(serial PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
