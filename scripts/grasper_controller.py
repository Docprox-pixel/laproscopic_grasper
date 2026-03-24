#!/usr/bin/env python3
"""
Grasper Motion Controller — smooth cubic-spline trajectory
Robot spawned at x=-1.00 on blue table (z=0.90 m).
Liver organ at x=+0.50, y=0.0, z=1.00 m (inside abdominal cavity).
Gear mechanism: drive pinion + worm screw + racks (visual in URDF).
Updated for 7-DOF KUKA iiwa14 arm.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Float32

import time
import threading
from builtin_interfaces.msg import Duration


class GrasperMotionController(Node):

    def __init__(self):
        super().__init__("grasper_motion_controller")
        self.get_logger().info("=" * 60)
        self.get_logger().info("  Grasper Motion Controller  —  ONLINE (7-DOF KUKA)")
        self.get_logger().info("=" * 60)

        self.arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.jaw_client = ActionClient(
            self, FollowJointTrajectory, "/grasper_controller/follow_joint_trajectory"
        )

        self.create_subscription(String,  "/surgical_robot/safety_alert", self._safety_cb, 10)
        self.create_subscription(Float32, "/surgical_robot/grip_force",   self._force_cb,  10)

        self.emergency_stop = False
        self.current_grip   = 0.0

        self.arm_joints = [
            "iiwa14_joint1", "iiwa14_joint2",
            "iiwa14_joint3", "iiwa14_joint4",
            "iiwa14_joint5", "iiwa14_joint6",
            "iiwa14_joint7"
        ]
        self.grasper_joints = [
            "grasper_pitch_joint",
            "grasper_yaw_joint",
            "grasper_jaw_joint",
        ]

        # Toggling off demo sequence to allow Gesture Control dominance
        # t = threading.Timer(5.0, self._run_sequence)
        # t.daemon = True
        # t.start()
        # self.get_logger().info("Sequence disabled (Gesture Control Mode active)")

    # ─── callbacks ────────────────────────────────────────────────────────
    def _safety_cb(self, msg: String):
        if "CRITICAL" in msg.data:
            self.get_logger().error(f"E-STOP: {msg.data}")
            self.emergency_stop = True

    def _force_cb(self, msg: Float32):
        self.current_grip = msg.data

    # ─── helpers ──────────────────────────────────────────────────────────
    def _dur(self, t: float) -> Duration:
        s = int(t)
        return Duration(sec=s, nanosec=int((t - s) * 1e9))

    def _make_point(self, pos, vel, t):
        pt = JointTrajectoryPoint()
        pt.positions         = list(pos)
        pt.velocities        = list(vel)
        pt.time_from_start   = self._dur(t)
        return pt

    def send_arm(self, waypoints, velocities, times):
        if self.emergency_stop:
            return False
        if not self.arm_client.wait_for_server(timeout_sec=30.0):
            return False
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints
        for pos, vel, t in zip(waypoints, velocities, times):
            traj.points.append(self._make_point(pos, vel, t))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.arm_client.send_goal_async(goal)
        return True

    def send_grasper(self, waypoints, velocities, times):
        if self.emergency_stop:
            return False
        if not self.jaw_client.wait_for_server(timeout_sec=30.0):
            return False
        traj = JointTrajectory()
        traj.joint_names = self.grasper_joints
        for pos, vel, t in zip(waypoints, velocities, times):
            traj.points.append(self._make_point(pos, vel, t))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.jaw_client.send_goal_async(goal)
        return True

    # ─── grasper preset states ─────────────────────────────────────────
    G_HOME  = [ 0.0, 0.0, 0.0]
    G_OPEN  = [ 0.0, 0.0, 0.8]
    G_DOWN  = [-1.57, 0.0, 0.8]  # Pitch down 90 deg, jaws open — enters cavity

    _Z7 = [0.0] * 7
    _Z3 = [0.0] * 3

    P_HOME  = [0.0,  0.0,  0.0, 0.0,  0.0, 0.0, 0.0]
    # Reaching towards x=0.5, y=0.0 from y=0.9; hovered above cavity opening.
    P_HOVER = [0.0,  0.4,  0.0, -1.5, 0.0, 1.1, 0.0]
    # Descend deeper into cavity to reach liver at z=1.00
    P_DOWN  = [0.0,  0.7,  0.0, -1.9, 0.0, 1.25, 0.0]
    # Lift: return to hover height post-extraction
    P_LIFT  = [0.0,  0.2,  0.0, -1.2, 0.0, 0.9, 0.0]

    def gentle_grasp(self, target_force=0.7):
        self.get_logger().info(f">>> GENTLE GRASP: Target {target_force}N")
        current_jaw = 0.8
        step = 0.04
        while current_jaw > 0.05:
            if self.current_grip >= target_force:
                self.get_logger().info(f">>> GRIP STABILIZED at {self.current_grip:.2f}N")
                return True
            current_jaw -= step
            self.send_grasper([[ -1.57, 0.0, current_jaw ]], [self._Z3], [0.2])
            time.sleep(0.2)
        self.get_logger().warn(">>> GRASP: Max closure reached without target force")
        return False

    def _run_sequence(self):
        try:
            self._sequence()
        except Exception as e:
            self.get_logger().error(f"Sequence error: {e}")

    # ─── MAIN SEQUENCE ────────────────────────────────────────────────────
    def _sequence(self):
        self.get_logger().info("Phase 1: Home & open grasper")
        self.send_arm([self.P_HOME], [self._Z7], [3.0])
        self.send_grasper([self.G_OPEN], [self._Z3], [3.0])
        time.sleep(3.5)

        self.get_logger().info("Phase 2: Pivot grasper downward & hover above cavity")
        self.send_grasper([self.G_DOWN], [self._Z3], [2.0])
        self.send_arm([self.P_HOVER], [self._Z7], [4.0])
        time.sleep(4.5)

        self.get_logger().info("Phase 3: Enter cavity and descend to liver (z≈1.00)")
        self.send_arm([self.P_DOWN], [self._Z7], [3.0])
        time.sleep(3.5)

        self.get_logger().info("Phase 4: Gentle Grasp with Force Feedback — target 0.8 N")
        self.gentle_grasp(target_force=0.8)
        time.sleep(1.0)

        self.get_logger().info("Phase 5: Lift liver OUT of the cavity")
        self.send_arm([self.P_LIFT], [self._Z7], [5.0])
        time.sleep(5.5)

        self.get_logger().info("Phase 6: Release liver")
        self.send_grasper([self.G_DOWN], [self._Z3], [2.0])
        time.sleep(2.5)

        self.get_logger().info("Phase 7: Return Home")
        self.send_arm([self.P_HOME], [self._Z7], [4.0])
        self.send_grasper([self.G_HOME], [self._Z3], [4.0])

        self.get_logger().info("Liver extraction sequence complete.")



def main(args=None):
    rclpy.init(args=args)
    node = GrasperMotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down …")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
