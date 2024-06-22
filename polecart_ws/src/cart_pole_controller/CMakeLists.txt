cmake_minimum_required(VERSION 3.5)
project(cart_pole_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Create the executable
add_executable(cart_pole_controller src/cart_pole_controller.cpp)

# Specify libraries to link against
ament_target_dependencies(cart_pole_controller rclcpp geometry_msgs sensor_msgs std_srvs)

# Install the executable
install(TARGETS
  cart_pole_controller
  DESTINATION lib/${PROJECT_NAME})

# Install configuration files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Ament package
ament_package()