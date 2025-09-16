cmake_minimum_required(VERSION 3.8)

project(ros_nt_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Make sure to specify the full path to the library files if they are not in a standard location.
# It's better to use absolute paths or relative paths from the project root.
# Here, I'm assuming the 'libs' directory is directly under the project root.
set(NTCORE_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libntcore.so")
set(NTCOREJNI_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libntcorejni.so")

# Create executable
add_executable(ros_nt_bridge src/main.cpp)

# Add ament dependencies
ament_target_dependencies(ros_nt_bridge rclcpp std_msgs)

# Link libraries
target_link_libraries(ros_nt_bridge
  PRIVATE
  ${NTCORE_LIBRARY}
  ${NTCOREJNI_LIBRARY}
)

# Install target
install(TARGETS ros_nt_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install the libraries. This is very important.
install(FILES
  ${NTCORE_LIBRARY}
  ${NTCOREJNI_LIBRARY}
  DESTINATION lib/${PROJECT_NAME}/libs # Install the libraries into a subdirectory
)

ament_package()