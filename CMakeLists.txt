cmake_minimum_required(VERSION 3.8)
project(ros_nt_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find ntcore (installed with WPILib)
find_library(NTCORE_LIB ntcore HINTS /usr/local/lib /usr/lib)
include_directories(/usr/local/include)  # adjust if WPILib is elsewhere

add_executable(ros_nt_bridge src/ros2_nt4_bridge.cpp)
ament_target_dependencies(ros_nt_bridge rclcpp std_msgs)
target_link_libraries(ros_nt_bridge ${NTCORE_LIB})

install(TARGETS ros_nt_bridge DESTINATION lib/${PROJECT_NAME})

ament_package()
