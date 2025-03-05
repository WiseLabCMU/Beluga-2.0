cmake_minimum_required(VERSION 3.22)
project(beluga)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Debugging
# add_compile_options(-g -O0)
add_definitions(
        # -DLOG_PUBS      # Log all messages being published
        # -DLOG_RANGES    # Log range reports
        # -DLOG_NEIGHBORS # Log neighbor updates
        # -DLOG_EXCHANGES # Log exchanges
        # -DTIMED_PUBLISHERS # All publishers are triggered timers
        # -DTIMED_NEIGHBOR_PUBLISHER # Publish neighbor updates on a timer
        # -DTIMED_RANGES_PUBLISHER # Publish range updates on a timer
        # -DTIMED_RANGE_EVENTS_PUBLISHER # Publish ranging events on a timer
)

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(beluga_messages REQUIRED)

set(BELUGA_SERIAL_DIR ../../../serial-comms/cpp/beluga-serial)

add_subdirectory(${BELUGA_SERIAL_DIR} beluga-serial-lib)

add_executable(beluga
        src/beluga_main.cpp
        src/beluga.cpp)
target_link_libraries(beluga beluga-serial)
ament_target_dependencies(beluga rclcpp beluga_messages)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
        beluga
        DESTINATION lib/${PROJECT_NAME})

ament_package()
