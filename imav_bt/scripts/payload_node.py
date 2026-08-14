#!/usr/bin/env python3
"""
payload_node.py — hardware abstraction for the mission payloads. MOCK BACKEND.

The tree talks to the payloads through services only, never to hardware
directly. That indirection is the point: none of the payload mechanisms exist
yet, and the ones that do get built will be driven by whatever the airframe
ends up using. Keeping the interface fixed means the mission subtrees can be
written, ticked and tested now.

Services offered (all std_srvs/Trigger, all currently mocked):

    /payload/drop_cone           release the 7 g PLA cone            (mission 3)
    /payload/blink_hotspot_led   blink the red LED                   (mission 3)
    /payload/grab_ring           close the grabber on the ring       (mission 4)
    /payload/release_ring        open the grabber                    (mission 4)
    /payload/read_continuity     report whether the circuit closed   (mission 4)

Two of these are worth reading the rulebook on before implementing:

  * ``blink_hotspot_led`` earns Hb=3 points on its own. Rulebook 4.4.6: the
    points are awarded "based SOLELY on this criterion" -- a red LED flashing
    visibly while the drone is directly above the correct box. It does not
    depend on the cone being dropped, which is why the tree blinks first.

  * ``read_continuity`` reports the buzzer/continuity result for mission 4's
    touchWind term (5 points). The contact system is explicitly left to each
    team by rulebook 4.4.7.

Implementing the real backend
-----------------------------
Set ``backend: px4`` (once written) and drive the hardware. The repo has no
GPIO or servo code today, but two paths already exist:

  * px4_msgs ships ActuatorServos.msg, Gripper.msg and LedControl.msg -- all
    currently unreferenced by any node in this workspace.
  * mobile_flight/mobile_gazebo/src/offboard_velocity_control.cpp:373 shows the
    working /fmu/in/vehicle_command pattern, which is how you would send
    VEHICLE_CMD_DO_SET_SERVO or DO_GRIPPER.

Keep the service names and semantics identical when you do, and nothing in the
tree needs to change.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

#: name -> (description, mock latency in seconds)
PAYLOAD_SERVICES = {
    "drop_cone": ("release the cone", 0.5),
    "blink_hotspot_led": ("blink the red hot-spot LED", 1.0),
    "grab_ring": ("close the grabber on the copper ring", 2.0),
    "release_ring": ("open the grabber", 1.0),
    "read_continuity": ("read the continuity / buzzer state", 0.2),
}


class PayloadNode(Node):
    """Serves the payload interface. With ``backend: mock``, nothing moves."""

    def __init__(self):
        super().__init__("payload_node")

        self.declare_parameter("backend", "mock")
        # Make any payload fail on demand, to rehearse a jammed servo or a
        # grabber that will not close, without touching the tree. Comma
        # separated, e.g. `-p fail:=grab_ring,drop_cone`.
        self.declare_parameter("fail", "")

        self.backend = self.get_parameter("backend").value
        self.failing = {
            name.strip()
            for name in str(self.get_parameter("fail").value).split(",")
            if name.strip()
        }

        if self.backend != "mock":
            raise NotImplementedError(
                f"backend {self.backend!r} is not implemented. Only 'mock' "
                f"exists today -- see this file's docstring for how to add a "
                f"real one."
            )

        self._services = [
            self.create_service(
                Trigger, f"/payload/{name}", self._handler(name, description)
            )
            for name, (description, _) in PAYLOAD_SERVICES.items()
        ]

        unknown = self.failing - set(PAYLOAD_SERVICES)
        if unknown:
            self.get_logger().warn(
                f"`fail` lists unknown payloads: {sorted(unknown)}; "
                f"known payloads are {sorted(PAYLOAD_SERVICES)}"
            )

        self.get_logger().info(
            f"payload_node ready | backend={self.backend} | "
            f"{len(self._services)} services"
            + (f" | forced failures: {sorted(self.failing)}" if self.failing else "")
        )

    def _handler(self, name, description):
        def callback(request, response):
            if name in self.failing:
                response.success = False
                response.message = f"MOCK: {name} forced to fail"
                self.get_logger().warn(response.message)
                return response

            response.success = True
            response.message = f"MOCK: {description} (no hardware attached)"
            self.get_logger().info(f"{name}: {response.message}")
            return response

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PayloadNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
