cmake_minimum_required(VERSION 3.8)
project(ros_nt_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ros_nt_bridge src/main.cpp)
ament_target_dependencies(ros_nt_bridge rclcpp std_msgs)
target_link_libraries(ros_nt_bridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libs/libntcore.so ${CMAKE_CURRENT_SOURCE_DIR}/libs/libntcorejni.so)

install(TARGETS ros_nt_bridge DESTINATION lib/${PROJECT_NAME})

ament_package()
