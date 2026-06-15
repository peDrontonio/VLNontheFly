cmake_minimum_required(VERSION 3.8)
project(planner)

find_package(ament_cmake REQUIRED)
file(GLOB_RECURSE IMPORT_FILES "imports/*.py")

# Instalar scripts Python como executáveis
install(PROGRAMS
  src/odometry_converter.py
  src/odometry_converter_optitrack.py
  src/point.py
  src/no_fmu.py
  src/iplanner.py
  src/iplanner_optitrack.py
  src/navdp.py
  src/controller.py
  src/controller_optitrack.py
  src/ego_planner_bridge.py
  src/pos_cmd_to_raptor.py
  src/relative_goal_to_map.py
  src/set_external_mode.py
  src/fig8_pos_cmd.py
  ${IMPORT_FILES}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
   imports/navdp/depth_anything
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY configs/
  DESTINATION share/${PROJECT_NAME}/configs
)

install(DIRECTORY
  checkpoints/
  DESTINATION share/${PROJECT_NAME}/checkpoints
)

# instala o pacote python
install(DIRECTORY 
  imports/iplanner/
  imports/navdp/
  imports/utils/
  DESTINATION lib/python3.10/site-packages/planner
)

# Instalar launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Trajectory2D.msg
  DEPENDENCIES geometry_msgs
)


ament_package()
