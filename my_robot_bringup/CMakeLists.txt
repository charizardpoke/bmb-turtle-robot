cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/led_status_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()