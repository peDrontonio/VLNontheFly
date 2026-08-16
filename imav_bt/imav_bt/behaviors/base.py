"""
base.py — base classes every imav_bt leaf inherits from.

Two of them, and picking the right one is most of the work of writing a new
behaviour:

    RosBehaviour   a leaf that may hold ROS state (publishers, subscriptions,
                   service clients) and can take several ticks to finish.
    Condition      a leaf that answers one yes/no question about the blackboard
                   and finishes in a single tick.

The setup(**kwargs) contract
----------------------------
py_trees calls setup() once, before the first tick, and passes through whatever
keyword arguments the tree owner supplied. py_trees_ros' convention -- which
this package follows -- is that the rclpy node arrives as `node`:

    tree.setup(node=my_rclpy_node, timeout=15.0)

RosBehaviour captures it and calls setup_ros(node), which is the hook subclasses
override. Subclasses that need no ROS at all (the stubs, most guards) set
``requires_node = False`` and can then be constructed and ticked in a unit test
with no ROS graph whatsoever. That is what keeps test/test_trees_smoke.py fast
and dependency-free, so please preserve it.

Never do ROS work in __init__
-----------------------------
Constructing a tree must be side-effect free: trees get built, inspected and
thrown away by tests and by `py-trees-tree-watcher`. Publishers, subscriptions
and service clients belong in setup_ros(), which only runs when someone
actually intends to fly.
"""

from __future__ import annotations

from typing import Any, Optional, Tuple

import py_trees
from py_trees.common import Status


class RosBehaviour(py_trees.behaviour.Behaviour):
    """A leaf that may own ROS entities and may run across several ticks.

    Attributes:
        node: the rclpy node, available from setup_ros() onwards. None until
            then, and None forever if ``requires_node`` is False.
    """

    #: When True, setup() raises if no node was supplied. Set False for leaves
    #: that are pure blackboard logic, so they stay unit-testable without ROS.
    requires_node: bool = True

    def __init__(self, name: str):
        super().__init__(name)
        self.node: Optional[Any] = None

    def setup(self, **kwargs: Any) -> None:
        """Capture the rclpy node and hand off to setup_ros()."""
        self.node = kwargs.get("node")
        if self.node is None and self.requires_node:
            raise KeyError(
                f"{self.name}: setup() needs a 'node' keyword argument. "
                f"Call tree.setup(node=<rclpy node>), or set "
                f"requires_node = False if this behaviour does not need ROS."
            )
        self.setup_ros(self.node)

    def setup_ros(self, node: Any) -> None:
        """Create publishers, subscriptions and clients here. Override in subclasses."""

    # -- logging helpers -------------------------------------------------
    # These fall back to py_trees' own logger when there is no ROS node, so a
    # behaviour logs identically whether it is flying or under unit test.

    def info(self, message: str) -> None:
        if self.node is not None:
            self.node.get_logger().info(f"[{self.name}] {message}")
        else:
            self.logger.info(f"[{self.name}] {message}")

    def warn(self, message: str) -> None:
        if self.node is not None:
            self.node.get_logger().warn(f"[{self.name}] {message}")
        else:
            self.logger.warning(f"[{self.name}] {message}")


class Condition(RosBehaviour):
    """A leaf that answers one yes/no question and never returns RUNNING.

    Subclasses implement :meth:`check`, returning ``(ok, reason)``. The reason
    string becomes the py_trees feedback message, so it shows up in
    ``py-trees-tree-watcher`` and in the failure logs -- which is the difference
    between "a guard failed" and "battery 18% below the 25% floor".

    Conditions default to ``requires_node = False`` because most of them only
    read the blackboard.
    """

    requires_node = False

    def check(self) -> Tuple[bool, str]:
        """Return (passed, human-readable reason). Override in subclasses."""
        raise NotImplementedError(f"{self.name}: Condition.check() not implemented")

    def update(self) -> Status:
        ok, reason = self.check()
        self.feedback_message = reason
        return Status.SUCCESS if ok else Status.FAILURE
