cmake_minimum_required(VERSION 3.8)

project(ros_nt_bridge)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17) # Or whichever standard you prefer
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Define library locations.  Use absolute paths or paths relative to the project root.
# This assumes 'libs' is a subdirectory in your project's root.
set(NTCORE_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libntcore.so")
set(NTCOREJNI_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libntcorejni.so")

# Create executable
add_executable(ros_nt_bridge src/main.cpp)

# Include directories (if needed by your header files)
target_include_directories(ros_nt_bridge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # For build
    $<INSTALL_INTERFACE:include> # For install
)

# Ament dependencies
ament_target_dependencies(ros_nt_bridge rclcpp std_msgs)

# Link libraries - USE KEYWORD ARGUMENTS ONLY
target_link_libraries(ros_nt_bridge
  PRIVATE
  ${NTCORE_LIBRARY}
  ${NTCOREJNI_LIBRARY}
)

# Install executable
install(TARGETS ros_nt_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(FILES
  ${NTCORE_LIBRARY}
  ${NTCOREJNI_LIBRARY}
  DESTINATION lib/${PROJECT_NAME}/libs
)

# Install header files (if any)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()