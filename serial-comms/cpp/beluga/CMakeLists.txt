cmake_minimum_required(VERSION 3.22)
add_library(beluga-lib)

include( FetchContent )
FetchContent_Declare(
        daw_json_link
        GIT_REPOSITORY https://github.com/beached/daw_json_link
        GIT_TAG release
)
FetchContent_MakeAvailable(daw_json_link)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/serial)

file(GLOB BELUGA_SRC PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)

target_sources(beluga-lib PRIVATE ${BELUGA_SRC})
target_include_directories(beluga-lib PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(beluga-lib PRIVATE daw::daw-json-link pthread serial)