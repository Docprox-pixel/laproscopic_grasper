#!/usr/bin/env python3
import cv2
import numpy as np
import mediapipe as mp
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class FullRobotGestureController(Node):
    def __init__(self):
        super().__init__('gesture_control_node')

        # connect to arm, grasper, and shaft trajectory action servers
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.grasper_client = ActionClient(self, FollowJointTrajectory, '/grasper_controller/follow_joint_trajectory')
        self.shaft_client = ActionClient(self, FollowJointTrajectory, '/shaft_controller/follow_joint_trajectory')

        self.arm_joints = [
            "iiwa14_joint1", "iiwa14_joint2", "iiwa14_joint3",
            "iiwa14_joint4", "iiwa14_joint5", "iiwa14_joint6",
            "iiwa14_joint7"
        ]
        self.grasper_joints = [
            "grasper_pitch_joint", "grasper_yaw_joint", "grasper_jaw_joint"
        ]

        # EMA smoothing — alpha controls how much each new value moves the output
        # lower = smoother but laggier, higher = more responsive but jittery
        self.alpha = 0.2
        self.smoothed_arm_joints = [0.0] * 7
        self.smoothed_grasper_jaw = 0.8  # starts open
        self.smoothed_shaft = 0.0        # starts retracted
        self.jaw_locked = False          # latches closed until an open gesture clears it

        # MediaPipe hands setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

        # open the default webcam
        self.cap = cv2.VideoCapture(0)
        
        # throttle commands so we don't flood the action server
        self.last_command_time = self.get_clock().now()
        self.command_rate = 0.1  # 10 Hz
        
        # drive the vision loop at 30 fps
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.get_logger().info("Full Robot Gesture Controller started.")

    def get_finger_flexion(self, hand_landmarks, finger_points):
        """Return a 0–1 flexion value based on how far the fingertip is from the base knuckle."""
        # finger_points: [base, pip, dip, tip]
        base = hand_landmarks[finger_points[0]]
        tip = hand_landmarks[finger_points[3]]
        
        # straight 2D distance in normalised image coords
        dist = np.sqrt((tip.x - base.x)**2 + (tip.y - base.y)**2)
        
        # rough calibration — ~0.05 when fully closed, ~0.4 when fully open
        # varies a bit with hand size and distance to camera, but good enough in practice
        raw_flexion = (dist - 0.1) / 0.3
        return np.clip(raw_flexion, 0.0, 1.0)

    def get_hand_rotation(self, hand_landmarks):
        """Calculate hand rotation from wrist to middle-finger MCP."""
        wrist = hand_landmarks[0]
        middle_mcp = hand_landmarks[9]
        
        angle = np.arctan2(middle_mcp.y - wrist.y, middle_mcp.x - wrist.x)
        # normalise to roughly [-1.5, 1.5] for use on joint 3 or 7
        return angle

    def get_hand_proximity(self, marks):
        """Map hand size (wrist-to-index-MCP distance) to shaft position (0 to 0.08 m)."""
        # wrist is landmark 0, index MCP is landmark 5
        dx = marks[0].x - marks[5].x
        dy = marks[0].y - marks[5].y
        dist = np.sqrt(dx*dx + dy*dy)
        # far away hand (~0.07) → retracted, close hand (~0.18) → fully extended
        ratio = (dist - 0.07) / (0.11) 
        return float(np.clip(ratio * 0.08, 0.0, 0.08))

    def is_thumbs_down_gesture(self, marks):
        """True when thumb is pointing down and the other fingers are all curled in."""
        # thumb tip (4) must be progressively lower than IP (3) and MCP (2)
        thumb_down = marks[4].y > marks[3].y and marks[3].y > marks[2].y
        # fingers 2–4 should all be retracted
        tips = [12, 16, 20]
        pips = [10, 14, 18]
        ext = [marks[t].y < marks[p].y for t, p in zip(tips, pips)]
        return thumb_down and sum(ext) == 0

    def get_vertical_shaft_pos(self, marks):
        """Map vertical wrist position to shaft extension (0 to 0.08 m)."""
        wrist_y = marks[0].y
        # hand at top of frame (y≈0.2) → full extension; at bottom (y≈0.7) → retracted
        ratio = (0.7 - wrist_y) / (0.7 - 0.2)
        return float(np.clip(ratio * 0.08, 0.0, 0.08))

    def is_three_fingers_gesture(self, marks):
        """Detect index, middle, and ring fingers extended."""
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        ext = [marks[t].y < marks[p].y for t, p in zip(tips, pips)]
        # three fingers = index(0), middle(1), ring(2) extended, pinky(3) down
        return ext[0] and ext[1] and ext[2] and not ext[3]

    def is_rock_on_gesture(self, marks):
        """Detect index and pinky extended."""
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        ext = [marks[t].y < marks[p].y for t, p in zip(tips, pips)]
        # rock on = index(0) and pinky(3) up, middle(1) and ring(2) down
        return ext[0] and ext[3] and not ext[1] and not ext[2]

    def is_point_gesture(self, marks):
        """Detect only index finger extended."""
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        ext = [marks[t].y < marks[p].y for t, p in zip(tips, pips)]
        return ext[0] and not ext[1] and not ext[2]

    def is_victory_gesture(self, marks):
        """Detect index and middle fingers extended."""
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        ext = [marks[t].y < marks[p].y for t, p in zip(tips, pips)]
        return ext[0] and ext[1] and not ext[2]

    def is_fist_gesture(self, marks):
        """Detect all fingers retracted."""
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        ext = [marks[t].y < marks[p].y for t, p in zip(tips, pips)]
        return sum(ext) <= 1

    def is_open_hand_gesture(self, marks):
        """Detect all fingers extended."""
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        ext = [marks[t].y < marks[p].y for t, p in zip(tips, pips)]
        return sum(ext) >= 4

    def timer_callback(self):
        if not rclpy.ok(): return
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Camera read failed!", throttle_duration_sec=2.0)
            return

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)

        if result.multi_hand_landmarks:
            for hand in result.multi_hand_landmarks:
                marks = hand.landmark
                
                # map wrist x/y to arm base and shoulder joints
                j1 = (marks[0].x - 0.5) * -4.0
                
                # shoulder joint tracks wrist height
                j2 = (marks[0].y - 0.5) * -2.5
                
                # arm roll follows hand rotation angle — clamped to avoid gimbal flip
                j3 = float(np.clip(self.get_hand_rotation(marks) * 2.0, -2.5, 2.5))
                
                # elbow flexion controlled by index finger bend
                index_flex = self.get_finger_flexion(marks, [5, 6, 7, 8])
                j4 = -2.0 + (index_flex * 2.0)  # range [-2, 0]
                
                # wrist roll from middle finger
                middle_flex = self.get_finger_flexion(marks, [9, 10, 11, 12])
                j5 = (middle_flex - 0.5) * 3.0  # range [-1.5, 1.5]
                
                # wrist pitch from ring finger
                ring_flex = self.get_finger_flexion(marks, [13, 14, 15, 16])
                j6 = (ring_flex - 0.5) * 2.0  # range [-1.0, 1.0]
                
                # wrist yaw from pinky
                pinky_flex = self.get_finger_flexion(marks, [17, 18, 19, 20])
                j7 = (pinky_flex - 0.5) * 3.0  # range [-1.5, 1.5]
                
                j7 = (pinky_flex - 0.5) * 3.0
                
                # Handle gripper logic with auto-latching
                if self.is_fist_gesture(marks):
                    self.jaw_locked = True
                    jaw_val = 0.0
                elif self.is_open_hand_gesture(marks):
                    self.jaw_locked = False
                    jaw_val = 0.8
                else:
                    if self.jaw_locked:
                        jaw_val = 0.0  # Keep gripper locked closed
                    else:
                        # Perform pinch if not locked
                        thumb_tip = marks[4]
                        index_tip = marks[8]
                        pinch_dist = np.sqrt((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2)
                        jaw_val = np.clip((pinch_dist - 0.05) / 0.2, 0.0, 0.8)

                # Control shaft depth with emergency retraction on thumbs-down
                shaft_val = self.smoothed_shaft
                lock_label = " [LOCKED]" if self.jaw_locked else ""
                
                if self.is_thumbs_down_gesture(marks):
                    shaft_target = 0.0
                    shaft_val = (self.alpha * shaft_target) + ((1.0 - self.alpha) * self.smoothed_shaft)
                    mode_label = "EMERGENCY RETRACT!"
                    mode_color = (0, 0, 255)
                    arm_lock = True
                    grasper_targets = [0.0, 0.0, jaw_val]
                elif self.is_victory_gesture(marks):
                    # SHAFT MODE: hand proximity controls insertion depth
                    shaft_target = self.get_hand_proximity(marks)
                    shaft_val = (self.alpha * shaft_target) + ((1.0 - self.alpha) * self.smoothed_shaft)
                    mode_label = "SHAFT MODE (VICTORY)"
                    mode_color = (0, 255, 255)
                    arm_lock = True  # lock arm while adjusting shaft
                    grasper_targets = [0.0, 0.0, jaw_val]
                elif self.is_point_gesture(marks):
                    # ROTATION MODE: point gesture lets you control grasper pitch/yaw
                    p_yaw = (marks[0].x - 0.5) * -2.0
                    p_pitch = (marks[0].y - 0.5) * -2.0
                    grasper_targets = [float(p_pitch), float(p_yaw), jaw_val]
                    mode_label = "ROTATION MODE (POINT)"
                    mode_color = (255, 0, 255)
                    arm_lock = True  # lock arm while in rotation mode
                elif self.is_rock_on_gesture(marks):
                    # WRIST MODE: rock-on gesture = fine wrist joint control
                    j5_target = (marks[0].x - 0.5) * -3.0
                    j6_target = (marks[0].y - 0.5) * -2.0
                    arm_targets = list(self.smoothed_arm_joints)
                    arm_targets[4] = j5_target
                    arm_targets[5] = j6_target
                    mode_label = "WRIST MODE (ROCK ON)"
                    mode_color = (0, 165, 255)
                    arm_lock = True  # only joints 5/6 move during wrist mode
                elif self.is_three_fingers_gesture(marks):
                    # THREE FINGERS: full 7-DOF arm tracking mode
                    mode_label = "ARM MODE (THREE FINGERS)"
                    mode_color = (255, 255, 255)
                    arm_lock = False
                    arm_targets = [j1, j2, j3, j4, j5, j6, j7]
                    grasper_targets = [0.0, 0.0, jaw_val]
                else:
                    mode_label = "ARM LOCKED (STANDBY)"
                    mode_color = (150, 150, 150) if not self.jaw_locked else (0, 100, 255)
                    arm_lock = True
                    grasper_targets = [0.0, 0.0, jaw_val]
                    if self.jaw_locked:
                        mode_label = "GRIP LOCKED"

                # make sure arm_targets is always defined before we try to use it
                if not arm_lock and 'arm_targets' not in locals():
                    arm_targets = [j1, j2, j3, j4, j5, j6, j7]

                self.smoothed_shaft = shaft_val

                # when arm is locked, freeze it at its last smoothed position
                if arm_lock:
                    arm_targets = self.smoothed_arm_joints  # freeze arm
                
                # smooth all targets with EMA
                for i in range(7):
                    self.smoothed_arm_joints[i] = (self.alpha * arm_targets[i]) + ((1.0 - self.alpha) * self.smoothed_arm_joints[i])
                
                self.smoothed_grasper_jaw = (self.alpha * jaw_val) + ((1.0 - self.alpha) * self.smoothed_grasper_jaw)

                # send at the configured rate limit
                now = self.get_clock().now()
                if (now - self.last_command_time).nanoseconds > self.command_rate * 1e9:
                    self.send_arm_goal(self.smoothed_arm_joints)
                    self.send_grasper_goal(grasper_targets)
                    self.send_shaft_goal([self.smoothed_shaft])
                    self.last_command_time = now

                self.mp_draw.draw_landmarks(frame, hand, self.mp_hands.HAND_CONNECTIONS)
                
                # overlay the surgical HUD at the bottom of the frame
                h, w, _ = frame.shape
                # semi-transparent black bar behind the text
                overlay = frame.copy()
                cv2.rectangle(overlay, (0, h-80), (w, h), (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
                
                # current mode label in the gesture colour
                cv2.putText(frame, f"MODE: {mode_label}", (20, h-45), cv2.FONT_HERSHEY_DUPLEX, 0.8, mode_color, 2)
                
                # live shaft and jaw values at the bottom
                stats = f"SHAFT: {self.smoothed_shaft*100:.1f}cm | JAW: {self.smoothed_grasper_jaw:.2f}"
                cv2.putText(frame, stats, (20, h-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        else:
            # show a warning banner when the hand goes out of frame
            h, w, _ = frame.shape
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (w, 60), (0, 0, 50), -1)
            cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
            cv2.putText(frame, "⚠️  SURGICAL PAUSE: NO HAND DETECTED", (30, 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Multi-Joint Gesture Control", frame)
        cv2.waitKey(1)

    def send_arm_goal(self, positions):
        if not self.arm_client.server_is_ready(): return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=0, nanosec=int(250 * 1e6))
        goal.trajectory.points.append(point)
        self.arm_client.send_goal_async(goal)

    def send_grasper_goal(self, positions):
        if not self.grasper_client.server_is_ready(): return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.grasper_joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=0, nanosec=int(250 * 1e6))
        goal.trajectory.points.append(point)
        self.grasper_client.send_goal_async(goal)

    def send_shaft_goal(self, positions):
        if not self.shaft_client.server_is_ready(): return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["shaft_translation_joint"]
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=0, nanosec=int(250 * 1e6))
        goal.trajectory.points.append(point)
        self.shaft_client.send_goal_async(goal)

    def shutdown_cleanly(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = FullRobotGestureController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_cleanly()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
