cmake_minimum_required(VERSION 2.8.3)
project(ri_sensor_scanner_3d)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  ri_platform_msgs
  roscpp
  tf
  ri_serial_driver
  ri_sensor_base
)

generate_dynamic_reconfigure_options(
  cfg/scanner_3d.cfg
)

catkin_package()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(scanner_3d src/scanner_3d.cpp)
target_link_libraries(scanner_3d
  ${catkin_LIBRARIES}
)

ADD_DEFINITIONS(
    -std=c++0x
)
