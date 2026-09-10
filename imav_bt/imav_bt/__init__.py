"""
imav_bt — behaviour-tree mission controller for the IMAV 2026 indoor competition.

This package is the decision-making layer that sits ABOVE the existing flight
stack. It does not fly the drone; it decides what the drone should be doing and
delegates the flying to ego-planner:

    imav_bt (this package)
      | publishes geometry_msgs/PoseStamped goals
      v
    /move_base_simple/goal  ->  ego_planner (ego_replan_fsm)
      | publishes std_msgs/String status on planning/goal_status
      | publishes traj_utils/Bspline
      v
    traj_server -> /drone_0_planning/pos_cmd (quadrotor_msgs/PositionCommand)
      v
    pos_cmd_to_raptor -> /fmu/in/trajectory_setpoint_raptor -> PX4 (EXTERNAL)

The tree reads pose from /odometry (published by planner's odometry_converter.py)
and goal feedback from planning/goal_status. It never writes actuator commands
directly -- that is deliberate, so the safety properties of the existing stack
(Raptor activation target, RC override, offboard failsafe) are untouched.

Layout
------
    blackboard.py    typed key schema shared by every behaviour
    bt_engine.py     tree assembly, ROS wiring, tick loop
    decorators/      policy decorators that encode the scoring strategy
    behaviors/       leaf behaviours (guards, flight, stubs, bookkeeping)
    trees/           tree and subtree assembly

Scope note
----------
The per-mission logic is intentionally NOT implemented here. Each mission
subtree in trees/ is a tickable placeholder with documented insertion points.
See docs/ADDING_A_MISSION.md.
"""

__all__ = ["blackboard", "bt_engine", "behaviors", "decorators", "trees"]
