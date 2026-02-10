cmake_minimum_required(VERSION 3.8)
project(simple_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position.msg"
  DEPENDENCIES std_msgs
)

# Install Python scripts
install(PROGRAMS
  simple_follower/scripts/laserTracker.py
  simple_follower/scripts/visualTracker.py
  simple_follower/scripts/follower.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()