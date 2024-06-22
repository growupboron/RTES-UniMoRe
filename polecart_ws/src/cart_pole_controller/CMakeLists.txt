cmake_minimum_required(VERSION 3.5)
project(cart_pole_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Create the regular PID controller executable
add_executable(cart_pole_controller src/cart_pole_controller.cpp)

# Create the Twiddle auto-tuning controller executable
add_executable(cart_pole_controller_tw src/cart_pole_controller_tw.cpp)

# Create the Ziegler-Nichols controller executable
add_executable(cart_pole_controller_zn src/cart_pole_controller_zn.cpp)

# Specify libraries to link against
ament_target_dependencies(cart_pole_controller rclcpp geometry_msgs sensor_msgs std_srvs)
ament_target_dependencies(cart_pole_controller_tw rclcpp geometry_msgs sensor_msgs std_srvs)
ament_target_dependencies(cart_pole_controller_zn rclcpp geometry_msgs sensor_msgs std_srvs)

# Install the executables
install(TARGETS
  cart_pole_controller
  cart_pole_controller_tw
  cart_pole_controller_zn
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