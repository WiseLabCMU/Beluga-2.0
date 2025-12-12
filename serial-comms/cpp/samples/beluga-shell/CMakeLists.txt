cmake_minimum_required(VERSION 3.22)
project(beluga-shell)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 23)

set(BELUGA_SERIAL_DIR ../../beluga-serial)

add_subdirectory(${BELUGA_SERIAL_DIR} beluga-serial-lib)

include_directories(include)

set(SRC_FILES
        src/main.c
)

add_executable(beluga-shell ${SRC_FILES})
target_link_libraries(beluga-serial)
