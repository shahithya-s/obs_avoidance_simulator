cmake_minimum_required(VERSION 3.8)
project(obs_avoidance_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(obstacle_avoidance_node src/obstacle_avoidance.cpp)

# Link libraries
ament_target_dependencies(obstacle_avoidance_node rclcpp geometry_msgs sensor_msgs)

# Install the executable
install(TARGETS
  obstacle_avoidance_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files (if any)
install(DIRECTORY launch urdf worlds
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py" PATTERN "*.urdf" PATTERN "*.world")

ament_package()
