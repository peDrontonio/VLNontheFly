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

# Depth-Anything-V2 is fetched, not vendored (see third_party.repos), and the
# NavDP/iPlanner checkpoints are large binaries that live outside git (see
# docs/MODELS.md). Both are only needed by the EXPERIMENTAL NavDP/iPlanner
# nodes, so their absence must not break a build of the validated flow.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/imports/navdp/depth_anything")
  install(DIRECTORY
    imports/navdp/depth_anything
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS
    "planner: imports/navdp/depth_anything not found -- skipping. "
    "Run `vcs import < third_party.repos` if you need the NavDP planner.")
endif()

install(DIRECTORY configs/
  DESTINATION share/${PROJECT_NAME}/configs
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/checkpoints")
  install(DIRECTORY
    checkpoints/
    DESTINATION share/${PROJECT_NAME}/checkpoints
  )
else()
  message(STATUS
    "planner: checkpoints/ not found -- skipping. "
    "Run scripts/fetch_models.sh if you need NavDP or iPlanner.")
endif()

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
