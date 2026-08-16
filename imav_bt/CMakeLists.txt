cmake_minimum_required(VERSION 3.8)
project(imav_bt)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Importable Python module: `from imav_bt.trees.root import build_root`.
# Same mechanism depth_estimator uses for its package module.
ament_python_install_package(${PROJECT_NAME})

# Executable nodes. House style (planner, edgellm_vlm_ros): keep the .py
# extension so launch files refer to `executable='bt_node.py'`.
install(PROGRAMS
  scripts/bt_node.py
  scripts/payload_node.py
  scripts/mock_arena.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
