#!/usr/bin/env python3
"""
bt_node.py — entry point for the behaviour-tree mission controller.

    ros2 run imav_bt bt_node.py
    ros2 run imav_bt bt_node.py --ros-args -p config_dir:=/path/to/config
    ros2 run imav_bt bt_node.py --ros-args -p flight_plan:=flight_b

Then, from another terminal::

    ros2 param set /imav_bt flight_plan flight_a       # choose the plan
    ros2 service call /imav_bt/set_running std_srvs/srv/SetBool "{data: true}"
    ros2 topic echo /imav_bt/snapshot --field data     # watch the tree
    ros2 service call /imav_bt/abort std_srvs/srv/Trigger  # stop it

All the logic lives in imav_bt/bt_engine.py; this file only exists because the
house packaging style installs executables as scripts (see CMakeLists.txt).
"""

from imav_bt.bt_engine import main

if __name__ == "__main__":
    main()
