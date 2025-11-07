cmake_minimum_required(VERSION 3.22)
add_library(beluga-serial)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/extern/daw_json_link)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/serial)

file(GLOB BELUGA_SRC PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)

target_sources(beluga-serial PRIVATE ${BELUGA_SRC})
target_include_directories(beluga-serial PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(beluga-serial PUBLIC daw::daw-json-link pthread serial)
