import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

import time


class TissueInteractionMonitor(Node):

    TISSUE_PROFILES = {
        "liver":       {"safe": 1.5, "warning": 2.0, "critical": 2.8},
        "intestine":   {"safe": 0.8, "warning": 1.2, "critical": 1.8},
        "gallbladder": {"safe": 1.0, "warning": 1.5, "critical": 2.2},
        "fat":         {"safe": 2.0, "warning": 2.8, "critical": 3.5},
    }

    def __init__(self):
        super().__init__("tissue_interaction_monitor")

        self.declare_parameter("tissue_type", "gallbladder")
        tissue_type = self.get_parameter("tissue_type").value

        if tissue_type not in self.TISSUE_PROFILES:
            self.get_logger().warn(
                f"Unknown tissue type '{tissue_type}', falling back to 'gallbladder'"
            )
            tissue_type = "gallbladder"

        self.profile = self.TISSUE_PROFILES[tissue_type]

        self.get_logger().info("=" * 55)
        self.get_logger().info("  Tissue Interaction Monitor - ONLINE")
        self.get_logger().info(f"  Tissue type:    {tissue_type.upper()}")
        self.get_logger().info(f"  Safe force:     {self.profile['safe']} N")
        self.get_logger().info(f"  Warning force:  {self.profile['warning']} N")
        self.get_logger().info(f"  Critical force: {self.profile['critical']} N")
        self.get_logger().info("=" * 55)

        self.current_force      = 0.0
        self.max_force_seen     = 0.0
        self.session_max        = 0.0
        self.contact_start_time = None
        self.interaction_events = []
        self.in_contact         = False

        # Subscribers
        self.create_subscription(Float32, "/surgical_robot/grip_force",   self.force_cb, 10)
        self.create_subscription(String,  "/surgical_robot/safety_alert", self.alert_cb, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, "/surgical_robot/interaction_report", 10)

        self.create_timer(0.1, self.monitor_interaction)  # 10 Hz
        self.create_timer(5.0, self.print_summary)        # 0.2 Hz

    def force_cb(self, msg: Float32):
        self.current_force = msg.data
        if msg.data > self.max_force_seen:
            self.max_force_seen = msg.data
        if self.in_contact and msg.data > self.session_max:
            self.session_max = msg.data

    def alert_cb(self, msg: String):
        if "CRITICAL" in msg.data:
            event = {
                "time":  time.time(),
                "type":  "CRITICAL_FORCE",
                "force": self.current_force,
                "msg":   msg.data,
            }
            self.interaction_events.append(event)
            self.get_logger().error(f"CRITICAL EVENT: {msg.data}")

    def monitor_interaction(self):
        CONTACT_THRESHOLD = 0.05  # N

        if self.current_force > CONTACT_THRESHOLD:
            if not self.in_contact:
                self.in_contact         = True
                self.contact_start_time = time.time()
                self.session_max        = self.current_force
                self.get_logger().info(
                    f"Contact START - Force: {self.current_force:.3f} N"
                )
        else:
            if self.in_contact:
                # FIX #8: Capture all session data BEFORE resetting state.
                # Original code reset session_max before using it in the log,
                # and the None-guard ran after state was partially torn down.
                # Now: snapshot → record event → log → reset state.

                # Snapshot current session data
                peak_force          = self.session_max
                contact_start_snap  = self.contact_start_time

                if contact_start_snap is None:
                    # Defensive: should never happen, but guard cleanly
                    self.get_logger().error("Contact end without recorded start - skipping event.")
                    self.in_contact = False
                    self.session_max = 0.0
                    return

                duration = time.time() - contact_start_snap

                # Record event using snapshotted values
                event = {
                    "type":       "contact_complete",
                    "duration_s": duration,
                    "peak_force": peak_force,
                    "quality":    self.assess_quality(peak_force),
                }
                self.interaction_events.append(event)

                self.get_logger().info(
                    f"Contact END - Duration: {duration:.1f}s | "
                    f"Peak force: {peak_force:.3f} N"
                )

                # Reset state AFTER event is fully recorded
                self.in_contact         = False
                self.contact_start_time = None
                self.session_max        = 0.0

        if self.in_contact:
            quality    = self.assess_quality(self.current_force)
            duration_s = time.time() - self.contact_start_time if self.contact_start_time else 0.0
            status      = String()
            status.data = (
                f"IN_CONTACT | Force: {self.current_force:.3f}N | "
                f"Quality: {quality} | Duration: {duration_s:.1f}s"
            )
            self.status_pub.publish(status)

    def assess_quality(self, force: float) -> str:
        if force <= 0.05:
            return "NO_CONTACT"
        elif force <= self.profile["safe"] * 0.5:
            return "EXCELLENT - Very gentle"
        elif force <= self.profile["safe"]:
            return "GOOD - Safe range"
        elif force <= self.profile["warning"]:
            return "WARNING - Reduce force"
        elif force <= self.profile["critical"]:
            return "DANGER - Tissue at risk"
        else:
            return "CRITICAL - Damage likely!"

    def print_summary(self):
        self.get_logger().info(
            f"\n[TISSUE MONITOR SUMMARY]\n"
            f"  Current force:    {self.current_force:.3f} N\n"
            f"  Max force seen:   {self.max_force_seen:.3f} N\n"
            f"  Safe limit:       {self.profile['safe']} N\n"
            f"  In contact:       {self.in_contact}\n"
            f"  Total events:     {len(self.interaction_events)}\n"
            f"  Quality:          {self.assess_quality(self.current_force)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TissueInteractionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Tissue monitor shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
