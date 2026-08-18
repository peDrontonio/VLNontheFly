cmake_minimum_required(VERSION 3.8)
project(planner)

find_package(ament_cmake REQUIRED)

# Instalar scripts Python como executáveis
install(PROGRAMS
  src/odometry_converter.py
  src/pos_cmd_to_raptor.py
  src/relative_goal_to_map.py
  src/set_external_mode.py
  src/fig8_pos_cmd.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
