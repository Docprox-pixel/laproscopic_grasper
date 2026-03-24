#!/usr/bin/env python3
"""
Grasper Command Bridge
=======================
Subscribes to /grasper/command (std_msgs/Float32, range 0.0-1.0)
and translates to FollowJointTrajectory action on the grasper_controller.

  0.0 = jaws fully CLOSED
  1.0 = jaws fully OPEN  (jaw joint ~0.8 rad)

Also publishes /grasper/state with the current jaw angle (Float32, radians).

Quick control examples:
  # Open
  ros2 topic pub /grasper/command std_msgs/Float32 "data: 1.0" --once
  # Half-open
  ros2 topic pub /grasper/command std_msgs/Float32 "data: 0.5" --once
  # Close
  ros2 topic pub /grasper/command std_msgs/Float32 "data: 0.0" --once
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float32, String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


# ──────────────────────────────────────────────────────────────────
#  Constants
# ──────────────────────────────────────────────────────────────────
JAW_JOINT        = "grasper_jaw_joint"          # primary jaw joint
PITCH_JOINT      = "grasper_pitch_joint"
YAW_JOINT        = "grasper_yaw_joint"
JAW_MAX_RAD      = 0.8                          # fully open (rad)
MOTION_DURATION  = 0.5                          # seconds for open/close


class GrasperCommandNode(Node):
    """Translates a normalised [0..1] command into jaw trajectory goals."""

    def __init__(self):
        super().__init__("grasper_command_node")

        self.get_logger().info("=" * 52)
        self.get_logger().info("  Grasper Command Bridge  —  ONLINE")
        self.get_logger().info("  /grasper/command  → trajectory action")
        self.get_logger().info("=" * 52)

        # Action client
        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            "/grasper_controller/follow_joint_trajectory"
        )

        # Current state
        self._current_jaw_rad  = 0.0
        self._current_pitch    = 0.0
        self._current_yaw      = 0.0
        self._goal_in_flight   = False

        # Subscriptions
        self.create_subscription(Float32,    "/grasper/command", self._cmd_cb,    10)
        self.create_subscription(JointState, "/joint_states",    self._state_cb,  10)

        # Publishers
        self._state_pub   = self.create_publisher(Float32, "/grasper/state",  10)
        self._status_pub  = self.create_publisher(String,  "/grasper/status", 10)

        # 10 Hz state republisher
        self.create_timer(0.1, self._publish_state)

        self.get_logger().info("Waiting for grasper_controller action server…")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server ready. /grasper/command is LIVE.")

    # ── State tracking ──────────────────────────────────────────────
    def _state_cb(self, msg: JointState):
        if len(msg.position) != len(msg.name):
            return
        for name, pos in zip(msg.name, msg.position):
            if name == JAW_JOINT:
                self._current_jaw_rad = pos
            elif name == PITCH_JOINT:
                self._current_pitch = pos
            elif name == YAW_JOINT:
                self._current_yaw = pos

    def _publish_state(self):
        state_msg = Float32()
        state_msg.data = float(self._current_jaw_rad)
        self._state_pub.publish(state_msg)

    # ── Command handler ─────────────────────────────────────────────
    def _cmd_cb(self, msg: Float32):
        """Clamp input to [0,1], scale to jaw radians, send trajectory."""
        normalised = max(0.0, min(1.0, msg.data))
        target_rad = normalised * JAW_MAX_RAD

        direction = "OPEN" if normalised > 0.5 else "CLOSE"
        self.get_logger().info(
            f"Command: {normalised:.2f}  →  jaw = {target_rad:.3f} rad  [{direction}]"
        )

        # Publish a human-readable status
        status = String()
        status.data = f"{direction} ({normalised*100:.0f}%)  jaw={target_rad:.3f} rad"
        self._status_pub.publish(status)

        self._send_goal(target_rad)

    def _send_goal(self, jaw_rad: float):
        traj = JointTrajectory()
        traj.joint_names = [PITCH_JOINT, YAW_JOINT, JAW_JOINT]

        pt = JointTrajectoryPoint()
        pt.positions  = [self._current_pitch, self._current_yaw, jaw_rad]
        pt.velocities = [0.0, 0.0, 0.0]

        secs = int(MOTION_DURATION)
        nsecs = int((MOTION_DURATION - secs) * 1e9)
        pt.time_from_start = Duration(sec=secs, nanosec=nsecs)

        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by grasper_controller")
            return
        self.get_logger().debug("Grasper goal accepted")


def main(args=None):
    rclpy.init(args=args)
    node = GrasperCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Grasper command node shutting down…")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
