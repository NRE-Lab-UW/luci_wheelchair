cmake_minimum_required(VERSION 3.8)
project(luci_encoders)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install scripts
install(PROGRAMS
  luci_encoders/encoder_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
