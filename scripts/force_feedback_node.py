#!/usr/bin/env python3
"""
Force Feedback Sensor Node
===========================
Monitors force/torque data from the laparoscopic grasper sensors.

FIX #7: Force thresholds were inconsistent across files.
  - README says: WARNING > 1.5N, CRITICAL > 2.0N
  - tissue_interaction.py gallbladder profile: safe=1.0, warning=1.5, critical=2.2
  - This file originally had: WARNING=1.5, CRITICAL=2.5, SAFE_MAX=2.0 (CRITICAL > SAFE_MAX is contradictory)
  Fixed: FORCE_WARNING=1.5, FORCE_CRITICAL=2.0 to match README and be consistent with SAFE_MAX.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

import math


class ForceFeedbackNode(Node):

    FORCE_WARNING  = 1.5   # N — matches README and tissue_interaction gallbladder warning
    FORCE_CRITICAL = 2.0   # N — matches README CRITICAL threshold (was 2.5, contradicting SAFE_MAX=2.0)
    FORCE_SAFE_MAX = 2.0   # N — maximum safe operating force

    def __init__(self):
        super().__init__("force_feedback_node")

        self.get_logger().info("=" * 55)
        self.get_logger().info("  Force Feedback Sensor Node - ONLINE")
        self.get_logger().info("  Laparoscopic Grasper Surgical Robot")
        self.get_logger().info("=" * 55)

        self.shaft_force  = [0.0, 0.0, 0.0]
        self.shaft_torque = [0.0, 0.0, 0.0]
        self.jaw_l_effort = 0.0
        self.jaw_r_effort = 0.0
        self.grip_force   = 0.0
        self.alert_active = False
        self.prev_severity = "OK"

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.create_subscription(WrenchStamped, "/surgical_robot/shaft_force_torque",
                                 self.shaft_ft_callback, qos)
        self.create_subscription(JointState, "/joint_states",
                                 self.joint_states_callback, 10)

        # Publishers
        self.grip_force_pub    = self.create_publisher(Float32,        "/surgical_robot/grip_force",    10)
        self.safety_alert_pub  = self.create_publisher(String,         "/surgical_robot/safety_alert",  10)
        self.tissue_status_pub = self.create_publisher(String,         "/surgical_robot/tissue_status", 10)
        self.feedback_pub      = self.create_publisher(WrenchStamped,  "/surgical_robot/force_feedback",10)
        self.diagnostics_pub   = self.create_publisher(DiagnosticArray, "/diagnostics",                 10)

        self.create_timer(0.02, self.process_feedback)    # 50 Hz
        self.create_timer(1.0,  self.publish_diagnostics) # 1 Hz

        self.get_logger().info("Force feedback node ready. Monitoring sensors...")

    def shaft_ft_callback(self, msg: WrenchStamped):
        f = msg.wrench.force
        t = msg.wrench.torque
        self.shaft_force  = [f.x, f.y, f.z]
        self.shaft_torque = [t.x, t.y, t.z]

    def joint_states_callback(self, msg: JointState):
        # Guard against empty effort array (common in Gazebo Harmonic)
        if len(msg.effort) != len(msg.name):
            return
        for i, name in enumerate(msg.name):
            if name == "jaw_left_joint":
                self.jaw_l_effort = abs(msg.effort[i])
            elif name == "jaw_right_joint":
                self.jaw_r_effort = abs(msg.effort[i])

    def process_feedback(self):
        jaw_avg      = (self.jaw_l_effort + self.jaw_r_effort) / 2.0
        shaft_mag    = math.sqrt(sum(f**2 for f in self.shaft_force))
        self.grip_force = jaw_avg + shaft_mag * 0.1

        gf_msg = Float32()
        gf_msg.data = float(self.grip_force)
        self.grip_force_pub.publish(gf_msg)

        # Determine current severity
        if self.grip_force >= self.FORCE_CRITICAL:
            severity = "CRITICAL"
        elif self.grip_force >= self.FORCE_WARNING:
            severity = "WARNING"
        else:
            severity = "OK"

        alert_msg  = String()
        tissue_msg = String()

        if severity == "CRITICAL":
            alert_msg.data  = f"CRITICAL: Grip force {self.grip_force:.2f}N - tissue damage risk!"
            tissue_msg.data = "DANGER - RELEASE GRIP"
            if self.prev_severity != "CRITICAL":
                self.get_logger().error(f"CRITICAL FORCE: {self.grip_force:.3f} N")
        elif severity == "WARNING":
            alert_msg.data  = f"WARNING: Grip force {self.grip_force:.2f}N - approaching limit"
            tissue_msg.data = "CAUTION - Reduce grip"
            if self.prev_severity == "OK":
                self.get_logger().warn(f"WARNING force: {self.grip_force:.3f} N")
        else:
            alert_msg.data  = f"OK: Grip force {self.grip_force:.2f}N - within safe range"
            tissue_msg.data = "SAFE - Normal operation"

        self.prev_severity = severity
        self.safety_alert_pub.publish(alert_msg)
        self.tissue_status_pub.publish(tissue_msg)

        # Re-publish processed F/T
        fb = WrenchStamped()
        fb.header.stamp    = self.get_clock().now().to_msg()
        fb.header.frame_id = "force_sensor_ring"
        fb.wrench.force.x  = self.shaft_force[0]
        fb.wrench.force.y  = self.shaft_force[1]
        fb.wrench.force.z  = self.shaft_force[2]
        fb.wrench.torque.x = self.shaft_torque[0]
        fb.wrench.torque.y = self.shaft_torque[1]
        fb.wrench.torque.z = self.shaft_torque[2]
        self.feedback_pub.publish(fb)

    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name        = "Surgical Force Feedback System"
        status.hardware_id = "laproscopic_grasper"

        if self.grip_force >= self.FORCE_CRITICAL:
            status.level   = DiagnosticStatus.ERROR
            status.message = f"CRITICAL force: {self.grip_force:.3f} N"
        elif self.grip_force >= self.FORCE_WARNING:
            status.level   = DiagnosticStatus.WARN
            status.message = f"WARNING force: {self.grip_force:.3f} N"
        else:
            status.level   = DiagnosticStatus.OK
            status.message = f"Normal: {self.grip_force:.3f} N"

        status.values = [
            KeyValue(key="grip_force_N",     value=f"{self.grip_force:.4f}"),
            KeyValue(key="jaw_left_effort",  value=f"{self.jaw_l_effort:.4f}"),
            KeyValue(key="jaw_right_effort", value=f"{self.jaw_r_effort:.4f}"),
            KeyValue(key="shaft_fx",         value=f"{self.shaft_force[0]:.4f}"),
            KeyValue(key="shaft_fy",         value=f"{self.shaft_force[1]:.4f}"),
            KeyValue(key="shaft_fz",         value=f"{self.shaft_force[2]:.4f}"),
            KeyValue(key="safe_limit_N",     value=f"{self.FORCE_SAFE_MAX}"),
            KeyValue(key="severity",         value=self.prev_severity),
        ]

        diag_array.status.append(status)
        self.diagnostics_pub.publish(diag_array)

        self.get_logger().info(
            f"[FORCE SENSOR] Grip: {self.grip_force:.3f}N | "
            f"Shaft: [{self.shaft_force[0]:.2f}, {self.shaft_force[1]:.2f}, "
            f"{self.shaft_force[2]:.2f}]N | "
            f"Jaws: L={self.jaw_l_effort:.3f} R={self.jaw_r_effort:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ForceFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Force feedback node shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
