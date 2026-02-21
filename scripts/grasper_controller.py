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
        self.get_logger().info("  Grasper Motion Controller  —  ONLINE")
        self.get_logger().info("  Robot at x=-1.00 m  |  Gear-driven grasper")
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
            "joint1_yaw", "joint2_shoulder",
            "joint3_elbow", "joint4_wrist",
        ]
        self.grasper_joints = [
            "joint5_wrist_pitch",
            "finger_left_proximal_joint",
            "finger_right_proximal_joint",
            "finger_left_distal_joint",
            "finger_right_distal_joint",
        ]

        t = threading.Timer(5.0, self._run_sequence)
        t.daemon = True
        t.start()
        self.get_logger().info("Sequence starts in 5 s …")

    # callbacks
    def _safety_cb(self, msg: String):
        if "CRITICAL" in msg.data:
            self.get_logger().error(f"E-STOP: {msg.data}")
            self.emergency_stop = True

    def _force_cb(self, msg: Float32):
        self.current_grip = msg.data

    # helpers 
    def _dur(self, t: float) -> Duration:
        s = int(t)
        return Duration(sec=s, nanosec=int((t - s) * 1e9))

    def _make_point(self, pos, vel, t):
        """Build a JointTrajectoryPoint with position + velocity (for cubic spline)."""
        pt = JointTrajectoryPoint()
        pt.positions         = list(pos)
        pt.velocities        = list(vel)
        pt.time_from_start   = self._dur(t)
        return pt

    def send_arm(self, waypoints, velocities, times):
        """
        waypoints  : list of [yaw, shoulder, elbow, wrist_roll]
        velocities : list of [dyaw, dshoulder, delbow, dwrist]
                     supply [0,0,0,0] at start/end, intermediate for smooth spline
        times      : list of float seconds from start
        """
        if self.emergency_stop:
            self.get_logger().error("E-STOP - arm blocked")
            return False
        if not self.arm_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Arm action server timeout")
            return False
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints
        for pos, vel, t in zip(waypoints, velocities, times):
            traj.points.append(self._make_point(pos, vel, t))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().warn("Arm trajectory rejected")
            return False
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=60.0)
        self.get_logger().info("Arm move complete")
        return True

    def send_grasper(self, waypoints, velocities, times):
        """
        Same signature as send_arm but for the 5-DOF grasper joints.
        [wrist_pitch, l_prox, r_prox, l_dist, r_dist]
        """
        if self.emergency_stop:
            self.get_logger().error("E-STOP - grasper blocked")
            return False
        if not self.jaw_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Grasper action server timeout")
            return False
        traj = JointTrajectory()
        traj.joint_names = self.grasper_joints
        for pos, vel, t in zip(waypoints, velocities, times):
            traj.points.append(self._make_point(pos, vel, t))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.jaw_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().warn("Grasper trajectory rejected")
            return False
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=40.0)
        self.get_logger().info("Grasper move complete")
        return True

    #  grasper preset states 
    # [wrist_pitch, l_prox, r_prox, l_dist, r_dist]
    G_HOME  = [ 0.00,  0.00,  0.00,  0.00,  0.00]
    G_OPEN  = [ 0.00,  0.50, -0.50,  0.30, -0.30]
    G_PITCH = [ 0.40,  0.50, -0.50,  0.30, -0.30]  # angled down toward tissue
    G_PINCH = [ 0.40,  0.10, -0.10,  0.08, -0.08]
    G_GRASP = [ 0.40,  0.05, -0.05,  0.04, -0.04]

    # zero-vel helpers
    _Z4 = [0.0, 0.0, 0.0, 0.0]          # 4 DOF zero vel
    _Z5 = [0.0, 0.0, 0.0, 0.0, 0.0]     # 5 DOF zero vel

    def _run_sequence(self):
        try:
            self._sequence()
        except Exception as e:
            self.get_logger().error(f"Sequence error: {e}")

    # MAIN SEQUENCE 
    def _sequence(self):
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("  SURGICAL SEQUENCE — SMOOTH CUBIC-SPLINE MOTION")
        self.get_logger().info("  Robot base:   x=-1.00 m, z=0.90 m (blue table)")
        self.get_logger().info("  Tissue target: x=+0.50 m, z=1.45 m")
        self.get_logger().info("  Gear mechanism: worm → pinion → racks")
        self.get_logger().info("=" * 60)

        # ── Phase 1: Home & open grasper (gear open) ──────────────────────
        self.get_logger().info("\n  Phase 1: Home — arm zero, gear-open grasper")
        self.send_arm(
            [[0.0, 0.0, 0.0, 0.0]],
            [self._Z4],
            [3.0]
        )
        # Gear-open: worm drives pinion → racks push fingers apart smoothly
        self.send_grasper(
            [self.G_HOME, self.G_OPEN],
            [self._Z5,
             self._Z5],
            [1.0, 4.0]
        )
        time.sleep(1.0)
        if self.emergency_stop: return

        # ── Phase 2: RISE HIGH — smooth ramp up shoulder, elbow tucked ────
        # Generates gently accelerating motion (vel=0 start/end, mid vel set)
        # so joint motion feels organic rather than stepping between poses.
        self.get_logger().info("\n  Phase 2: Rise high — smooth shoulder lift")
        self.send_arm(
            [
                [0.0, 0.22, -0.06, 0.0],   # t=3  slow start
                [0.0, 0.52, -0.10, 0.0],   # t=6  accelerating
                [0.0, 0.78, -0.14, 0.0],   # t=9  near apex, decelerating
                [0.0, 0.90, -0.16, 0.0],   # t=12 apex — tip z≈1.65 m
            ],
            [
                [0.0,  0.08, -0.01, 0.0],  # gentle ramp-out vel
                [0.0,  0.12, -0.02, 0.0],  # peak travel vel
                [0.0,  0.06, -0.01, 0.0],  # decelerating
                self._Z4,                  # smooth stop at apex
            ],
            [3.0, 6.0, 9.0, 12.0]
        )
        time.sleep(0.3)
        if self.emergency_stop: return

        # Phase 3: YAW smooth swing to align with tissue 
        self.get_logger().info("\n  Phase 3: Smooth yaw swing → tissue alignment")
        self.send_arm(
            [
                [0.02, 0.90, -0.16, 0.0],
                [0.05, 0.90, -0.16, 0.0],
            ],
            [
                [0.005, 0.0, 0.0, 0.0],
                self._Z4,
            ],
            [3.0, 6.0]
        )
        time.sleep(0.3)
        if self.emergency_stop: return

        # Phase 4: ARC OVER TABLE — keep height, smoothly extend elbow 
        # Robot is now at x=-1.00. Patient table near-edge = x=-0.30, z=0.955.
        # Arm rises high first then extends, so tip arcs OVER the edge.
        self.get_logger().info("\n  Phase 4: Smooth arc over patient table edge")
        self.send_arm(
            [
                [0.05, 0.90, -0.28, 0.0],  # t=3   start reach — tip well above edge
                [0.05, 0.88, -0.48, 0.0],  # t=6   tip passes over table (z>1.25)
                [0.05, 0.86, -0.64, 0.0],  # t=9   tip above torso zone
                [0.05, 0.85, -0.74, 0.0],  # t=12  tip horizontally over tissue x
            ],
            [
                [0.0,  -0.01, -0.07, 0.0],  # slow start
                [0.0,  -0.01, -0.08, 0.0],  # peak travel vel
                [0.0,  -0.01, -0.05, 0.0],  # decelerating
                self._Z4,                   # stop over tissue
            ],
            [3.0, 6.0, 9.0, 12.0]
        )
        # Start pitching wrist down while arm extends (gear begins engaging)
        self.send_grasper(
            [
                self.G_OPEN,
                self.G_PITCH,
            ],
            [self._Z5, self._Z5],
            [2.0, 8.0]
        )
        time.sleep(0.3)
        if self.emergency_stop: return

        # Phase 5: DESCEND — vertical drop from directly above tissue 
        self.get_logger().info("\n  Phase 5: Smooth vertical descent onto tissue")
        self.send_arm(
            [
                [0.05, 0.83, -0.77, 0.12],  # t=3  start drop, shoulder dips slightly
                [0.05, 0.81, -0.79, 0.16],  # t=6  at tissue height z≈1.45 m
            ],
            [
                [0.0, -0.015, -0.01, 0.03],  # slow controlled descent
                self._Z4,                    # settle
            ],
            [3.0, 6.0]
        )
        time.sleep(0.5)
        if self.emergency_stop: return

        # Phase 6: GEAR-ENGAGE GRASP — worm drives racks inward 
        # Progressive 4-step close mimics worm-gear actuation: the fingers
        # don't snap shut but ratchet smoothly as the worm advances each rack.
        self.get_logger().info(f"\n  Phase 6: Gear-engage grasp  (pre-force: {self.current_grip:.3f} N)")
        self.send_grasper(
            [
                self.G_PITCH,                              # t=1  fully open
                [0.40,  0.38, -0.38,  0.22, -0.22],       # t=3  rack step 1
                [0.40,  0.22, -0.22,  0.15, -0.15],       # t=5  rack step 2
                self.G_PINCH,                              # t=7  rack step 3
                self.G_GRASP,                              # t=9  rack step 4 — closed
            ],
            [
                self._Z5,
                [0.0, -0.09,  0.09, -0.04,  0.04],  # smooth intermediate vel
                [0.0, -0.08,  0.08, -0.04,  0.04],
                [0.0, -0.06,  0.06, -0.03,  0.03],
                self._Z5,                             # smooth stop when closed
            ],
            [1.0, 3.0, 5.0, 7.0, 9.0]
        )
        time.sleep(1.5)
        self.get_logger().info(f"  Post-grasp force: {self.current_grip:.3f} N")

        if self.emergency_stop:
            self.get_logger().error("E-STOP after grasp — releasing via gear reverse!")
            self.send_grasper([self.G_OPEN], [self._Z5], [2.5])
            return

        # Phase 7: LIFT — rise straight up with tissue 
        self.get_logger().info("\n  Phase 7: Smooth lift — tissue rises with arm")
        self.send_arm(
            [
                [0.05, 0.84, -0.75, 0.12],  # t=3 lift — shoulder up
                [0.05, 0.87, -0.68, 0.08],  # t=6 clear torso / drape
            ],
            [
                [0.0,  0.012, 0.02, -0.015],  # upward vel
                self._Z4,
            ],
            [3.0, 6.0]
        )
        time.sleep(2.0)
        self.get_logger().info(f"  Holding tissue — force: {self.current_grip:.3f} N")
        if self.emergency_stop: return

        # Phase 8: GEAR-REVERSE RELEASE — racks retract outward 
        self.get_logger().info("\n  Phase 8: Gear-reverse release — racks open")
        self.send_grasper(
            [
                self.G_GRASP,                              # t=0.5 start
                self.G_PINCH,                              # t=2   step 1
                [0.40,  0.22, -0.22,  0.15, -0.15],       # t=4   step 2
                [0.40,  0.38, -0.38,  0.22, -0.22],       # t=6   step 3
                self.G_OPEN,                               # t=8   fully open
            ],
            [
                self._Z5,
                [0.0,  0.06, -0.06,  0.03, -0.03],
                [0.0,  0.08, -0.08,  0.04, -0.04],
                [0.0,  0.09, -0.09,  0.04, -0.04],
                self._Z5,
            ],
            [0.5, 2.0, 4.0, 6.0, 8.0]
        )
        time.sleep(1.0)

        # Phase 9: RETRACT — same high-arc path in reverse 
        self.get_logger().info("\n  Phase 9: Smooth high-arc retract to home")
        self.send_grasper(
            [self.G_OPEN, self.G_HOME],
            [self._Z5, self._Z5],
            [2.0, 5.0]
        )
        self.send_arm(
            [
                [0.05, 0.87, -0.55, 0.0],  # t=3   elbow tucks first
                [0.03, 0.90, -0.28, 0.0],  # t=6   back to apex height
                [0.02, 0.78, -0.14, 0.0],  # t=9   begin lowering
                [0.01, 0.52, -0.10, 0.0],  # t=12  pass table-edge height
                [0.00, 0.22, -0.06, 0.0],  # t=15  descending
                [0.00,  0.0,   0.0, 0.0],  # t=18  home
            ],
            [
                [0.0,  0.01,  0.09, 0.0],  # elbow retract vel
                [0.0, -0.01,  0.07, 0.0],  # shoulder rise vel
                [0.0, -0.06, -0.01, 0.0],  # shoulder lower vel
                [0.0, -0.08, -0.01, 0.0],
                [0.0, -0.06, -0.01, 0.0],
                self._Z4,                  # smooth stop at home
            ],
            [3.0, 6.0, 9.0, 12.0, 15.0, 18.0]
        )

        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("  SURGICAL SEQUENCE COMPLETE [OK]")
        self.get_logger().info("  Gear mechanism: worm retracted, racks at home")
        self.get_logger().info("=" * 60)


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
