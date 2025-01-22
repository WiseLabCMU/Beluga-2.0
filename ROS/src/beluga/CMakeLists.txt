cmake_minimum_required(VERSION 3.22)
project(beluga)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(beluga_messages REQUIRED)
find_package(daw-json-link)

file(GLOB SERIAL_SRC serial/*.cpp)
file(GLOB SERIAL_TOOLS serial/tools/*.cpp)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED))

add_executable(beluga
        src/beluga_main.cpp
        src/beluga.cpp
        src/beluga_frame.cpp
        ${SERIAL_SRC}
        ${SERIAL_TOOLS})
#target_link_libraries(beluga daw::daw_json_link)
ament_target_dependencies(beluga rclcpp beluga_messages daw-json-link)

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
