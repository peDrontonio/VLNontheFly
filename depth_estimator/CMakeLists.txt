cmake_minimum_required(VERSION 3.8)
project(depth_estimator)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Instala o módulo Python
ament_python_install_package(${PROJECT_NAME})

# Instala os scripts executáveis
install(PROGRAMS
  depth_estimator/depth_estimator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME depth_estimator_node
)

# Instala launch e config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
