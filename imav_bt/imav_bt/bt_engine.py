"""
bt_engine.py — the rclpy node that owns the tree.

Responsibilities, and deliberately nothing else:

  1. Load config/{bt,arena,missions}.yaml and build the tree.
  2. Subscribe to /odometry, planning/goal_status and the battery topic, and
     write those into the SENSED blackboard keys. This is the ONLY writer of
     those keys -- behaviours read them and must never fabricate them.
  3. Offer the operator controls: ~/set_running, ~/abort, and the `flight_plan`
     parameter.
  4. Tick the tree on a timer and publish status.

The engine holds no mission knowledge whatsoever. Adding a mission never
requires editing this file.


Why not py_trees_ros
--------------------
py_trees_ros provides tree introspection topics and the `py-trees-tree-watcher`
CLI, which are genuinely nice. It is an apt-only package
(`ros-humble-py-trees-ros`) and is NOT currently installed on this machine, so
depending on it would mean shipping code that cannot be run or tested here.

Instead this node publishes a rendered snapshot of the tree as a plain string on
``~/snapshot``, which covers the same need with no extra dependency::

    ros2 topic echo /imav_bt/snapshot --field data

If you later install py_trees_ros, swapping in its BehaviourTree is a change to
this file alone -- no behaviour or subtree is affected. See docs/ARCHITECTURE.md.


Threading note
--------------
Everything runs on the single-threaded default executor: subscriptions write
blackboard keys, then the timer ticks the tree. So a tick always sees a
self-consistent snapshot and no locking is needed anywhere in the behaviours.
Do not move the tick onto its own callback group without revisiting that.
"""

from __future__ import annotations

import math
import os
import time
from typing import Any, Dict, Optional, Tuple

import py_trees
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from imav_bt import blackboard as bb
from imav_bt import interfaces
from imav_bt.behaviors.bookkeeping import summary
from imav_bt.trees.root import build_root

CONFIG_FILES = ("bt.yaml", "arena.yaml", "missions.yaml")


def yaw_from_quaternion(q) -> float:
    """Extract ENU yaw from a geometry_msgs quaternion.

    Matches the helper in planner/src/relative_goal_to_map.py, so the tree and
    the goal adapter agree on what "yaw" means.
    """
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def load_config(config_dir: str) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """Load and merge the three YAML config files.

    Args:
        config_dir: directory holding bt.yaml, arena.yaml and missions.yaml.

    Returns:
        ``(config, arena)`` where ``config`` carries the ``safety``,
        ``missions``, ``landing``, ``topics``, ``tick_hz`` and
        ``snapshot_every_n_ticks`` keys, and ``arena`` is the anchor table.

    Raises:
        FileNotFoundError: if any of the three files is missing.
    """
    loaded: Dict[str, Dict[str, Any]] = {}
    for filename in CONFIG_FILES:
        path = os.path.join(config_dir, filename)
        if not os.path.isfile(path):
            raise FileNotFoundError(
                f"{path} not found. Point the `config_dir` parameter at a "
                f"directory containing {', '.join(CONFIG_FILES)}."
            )
        with open(path, "r") as handle:
            loaded[filename] = yaml.safe_load(handle) or {}

    bt_cfg = loaded["bt.yaml"]
    missions_cfg = loaded["missions.yaml"]

    config: Dict[str, Any] = {
        "tick_hz": float(bt_cfg.get("tick_hz", 10.0)),
        "snapshot_every_n_ticks": int(bt_cfg.get("snapshot_every_n_ticks", 10)),
        "topics": bt_cfg.get("topics") or {},
        "safety": bt_cfg.get("safety") or {},
        "missions": missions_cfg.get("missions") or {},
        "landing": missions_cfg.get("landing") or {},
    }
    return config, loaded["arena.yaml"]


class BtEngine(Node):
    """rclpy node that builds, feeds and ticks the behaviour tree."""

    def __init__(self):
        super().__init__("imav_bt")

        default_dir = self._default_config_dir()
        self.declare_parameter("config_dir", default_dir)
        self.declare_parameter("flight_plan", bb.FlightPlan.FLIGHT_A)
        # Convenience for bench runs: start ticking already armed, so no
        # service call is needed. NEVER set this true on the real aircraft.
        self.declare_parameter("autostart", False)

        config_dir = self.get_parameter("config_dir").value
        self.config, self.arena = load_config(config_dir)
        self.get_logger().info(f"config loaded from {config_dir}")

        # Blackboard first: the tree's constructors register clients against it.
        bb.reset()
        self._bb = bb.client(
            "engine",
            read=["safety_reason"],
            write=[
                "odom_valid", "odom_stamp", "position", "yaw",
                "goal_status", "goal_status_stamp",
                "battery_valid", "battery_fraction",
                "running", "flight_plan", "abort_requested",
            ],
        )
        self._bb.flight_plan = self._validated_plan(
            self.get_parameter("flight_plan").value
        )

        self.root = build_root(self.config, self.arena.get("anchors") or {})
        self.tree = py_trees.trees.BehaviourTree(self.root)
        # NOTE: py_trees.trees.setup(), not root.setup(). Behaviour.setup() does
        # not recurse, and setup_with_descendants() recurses but drops kwargs,
        # so neither delivers `node` to a leaf under a decorator.
        # See docs/TREE_SEMANTICS.md.
        py_trees.trees.setup(root=self.root, node=self)

        self._wire_ros()

        self._ticks = 0
        self._snapshot_every = self.config["snapshot_every_n_ticks"]
        tick_hz = self.config["tick_hz"]
        self.create_timer(1.0 / tick_hz, self._on_tick)

        if self.get_parameter("autostart").value:
            self._bb.running = True
            self.get_logger().warn(
                "autostart is TRUE — the tree is armed without an operator "
                "call. This is for bench runs only."
            )

        self.get_logger().info(
            f"imav_bt ready | {tick_hz:.1f} Hz | plan={self._bb.flight_plan} | "
            f"idle until ~/set_running is called"
        )

    # -- construction helpers ---------------------------------------------

    @staticmethod
    def _default_config_dir() -> str:
        """Installed share/config if available, else the source tree."""
        try:
            from ament_index_python.packages import get_package_share_directory

            return os.path.join(get_package_share_directory("imav_bt"), "config")
        except Exception:
            return os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config"
            )

    def _topic(self, key: str, default: str) -> str:
        return str(self.config["topics"].get(key, default))

    @staticmethod
    def _validated_plan(value: Any) -> str:
        plan = str(value)
        if plan not in bb.FlightPlan.ALL:
            raise ValueError(
                f"flight_plan must be one of {bb.FlightPlan.ALL}, got {plan!r}"
            )
        return plan

    def _wire_ros(self) -> None:
        # /odometry is published BEST_EFFORT by planner/src/odometry_converter.py;
        # a RELIABLE subscription would silently never connect to it.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Odometry, self._topic("odom", interfaces.TOPIC_ODOM),
            self._on_odom, sensor_qos,
        )
        self.create_subscription(
            String, self._topic("goal_status", interfaces.TOPIC_GOAL_STATUS),
            self._on_goal_status, 10,
        )
        self.create_subscription(
            BatteryState, self._topic("battery", interfaces.TOPIC_BATTERY),
            self._on_battery, sensor_qos,
        )

        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.snapshot_pub = self.create_publisher(String, "~/snapshot", 1)

        self.create_service(SetBool, "~/set_running", self._on_set_running)
        self.create_service(Trigger, "~/abort", self._on_abort)
        self.add_on_set_parameters_callback(self._on_set_parameters)

    # -- subscriptions: the ONLY writers of the SENSED keys ----------------

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._bb.position = (float(p.x), float(p.y), float(p.z))
        self._bb.yaw = float(yaw_from_quaternion(msg.pose.pose.orientation))
        self._bb.odom_valid = True
        # Receipt time, not the header stamp: a publisher that wedges while
        # still stamping fresh headers is a real failure mode we must catch.
        self._bb.odom_stamp = time.monotonic()

    def _on_goal_status(self, msg: String) -> None:
        self._bb.goal_status = str(msg.data).strip()
        self._bb.goal_status_stamp = time.monotonic()

    def _on_battery(self, msg: BatteryState) -> None:
        fraction = float(msg.percentage)
        # BatteryState.percentage is 0..1 per REP, but PX4 bridges sometimes
        # publish 0..100. Normalise rather than trip the guard at 100%.
        if fraction > 1.0:
            fraction /= 100.0
        if not math.isfinite(fraction):
            return
        self._bb.battery_fraction = max(0.0, min(1.0, fraction))
        self._bb.battery_valid = True

    # -- operator controls -------------------------------------------------

    def _on_set_running(self, request, response):
        if request.data:
            self._bb.abort_requested = False
            self._bb.running = True
            response.message = f"running: plan={self._bb.flight_plan}"
        else:
            self._bb.running = False
            # Clearing the abort latch here (and only here) means a tripped
            # abort cannot be shrugged off without a deliberate stop first.
            self._bb.abort_requested = False
            response.message = "stopped and abort latch cleared"
        response.success = True
        self.get_logger().info(response.message)
        return response

    def _on_abort(self, request, response):
        self._bb.abort_requested = True
        response.success = True
        response.message = "abort latched; safety branch takes over next tick"
        self.get_logger().warn(response.message)
        return response

    def _on_set_parameters(self, params):
        for param in params:
            if param.name != "flight_plan":
                continue
            try:
                plan = self._validated_plan(param.value)
            except ValueError as exc:
                return SetParametersResult(successful=False, reason=str(exc))
            if self._bb.running:
                return SetParametersResult(
                    successful=False,
                    reason="refusing to switch flight plan while running; "
                           "call ~/set_running false first",
                )
            self._bb.flight_plan = plan
            self.get_logger().info(f"flight plan set to {plan}")
        return SetParametersResult(successful=True)

    # -- the tick ----------------------------------------------------------

    def _on_tick(self) -> None:
        try:
            self.tree.tick()
        except Exception as exc:  # keep the node alive so safety stays reachable
            self.get_logger().error(f"tick raised {exc!r}; holding position")
            return

        self._ticks += 1

        message = String()
        reason = self._bb.safety_reason
        message.data = (
            f"tick={self._ticks} root={self.root.status.name} "
            f"{'SAFETY=' + reason + ' ' if reason else ''}| {summary()}"
        )
        self.status_pub.publish(message)

        if self._snapshot_every and self._ticks % self._snapshot_every == 0:
            snapshot = String()
            snapshot.data = py_trees.display.unicode_tree(
                self.root, show_status=True
            )
            self.snapshot_pub.publish(snapshot)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BtEngine()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
