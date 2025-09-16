cmake_minimum_required(VERSION 3.8)
project(ros_nt_bridge)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Define library locations
set(NTCORE_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libntcore.so")
set(NTCOREJNI_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libntcorejni.so")

# Create executable
add_executable(ros_nt_bridge src/main.cpp)

# Include directories
target_include_directories(ros_nt_bridge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link external libraries first (using plain signature)
target_link_libraries(ros_nt_bridge
    ${NTCORE_LIBRARY}
    ${NTCOREJNI_LIBRARY}
)

# Ament dependencies (also uses plain signature)
ament_target_dependencies(ros_nt_bridge rclcpp std_msgs)

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

# Install header files
install(DIRECTORY include/
    DESTINATION include
)

ament_package()