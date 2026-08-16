cmake_minimum_required(VERSION 3.8)
project(imav_bt)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Importable Python module: `from imav_bt.trees.root import build_root`.
# Same mechanism depth_estimator uses for its package module.
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
