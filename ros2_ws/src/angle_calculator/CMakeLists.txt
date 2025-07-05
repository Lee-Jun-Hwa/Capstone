cmake_minimum_required(VERSION 3.5)
project(angle_calculator)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})
ament_package()