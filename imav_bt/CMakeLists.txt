cmake_minimum_required(VERSION 3.8)
project(imav_bt)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

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
install(DIRECTORY config launch docs
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Plain unittest scripts run directly, matching edgellm_vlm_ros. These tests
  # deliberately do NOT require a ROS graph -- they tick trees against a fake
  # node, so they stay fast and runnable on any machine.
  add_test(NAME test_blackboard
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_blackboard.py)
  add_test(NAME test_decorators
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_decorators.py)
  add_test(NAME test_trees_smoke
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_trees_smoke.py)
  add_test(NAME test_flight
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_flight.py)

  # The tests import `imav_bt` straight from the source tree so they can run
  # before/without an install step.
  set_tests_properties(test_blackboard test_decorators test_trees_smoke test_flight
    PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}")
endif()

ament_package()
