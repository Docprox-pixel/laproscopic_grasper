#!/usr/bin/env python3
"""
Simple Surgical Robot Gesture Control

Five intuitive hand gestures mapped to robot actions:

  GESTURE                          EFFECT
  ──────────────────────────────── ──────────────────────────────────────────
  Move wrist LEFT / RIGHT          Rotate arm base (Joint 1)
  Move wrist UP / DOWN             Raise / lower arm (Joint 2)
  ✊ FIST (0-1 fingers)            Close grasper jaw
  🖐 OPEN HAND (4-5 fingers)       Open grasper jaw
  ✌ VICTORY (index + middle)      SHAFT SLIDING MODE
    └─ Move hand UP                → Extend shaft (insert)
    └─ Move hand DOWN              → Retract shaft (withdraw)
  ☝ POINT (index finger only)     GRASPER ROTATION MODE
    └─ Move hand L/R/U/D/Z         → Roll/Pitch/Yaw
  👎 THUMBS DOWN                  EMERGENCY SHAFT RETRACT
"""

import cv2
import numpy as np
import mediapipe as mp
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# EMA smoothing and trajectory timing
ALPHA      = 0.12   # EMA smoothing coefficient
CMD_HZ     = 8.0    # how often we send trajectory goals
TRAJ_MS    = 400    # milliseconds to complete each move

# shaft depth calibration using wrist.z from MediaPipe (relative metres, negative)
# bring the hand about 20 cm closer to the camera for full extension
SHAFT_Z_FAR  = -0.10   # wrist z when hand is comfortably far  → retracted (0.0 m)
SHAFT_Z_NEAR = -0.45   # wrist z when hand is pushed close     → fully extended (0.20 m)


class SimpleGestureController(Node):
    ARM_JOINTS     = ["iiwa14_joint1", "iiwa14_joint2", "iiwa14_joint3",
                      "iiwa14_joint4", "iiwa14_joint5", "iiwa14_joint6",
                      "iiwa14_joint7"]
    GRASPER_JOINTS = ["grasper_roll_joint", "grasper_pitch_joint", "grasper_yaw_joint", "grasper_jaw_joint"]
    SHAFT_JOINT    = ["shaft_translation_joint"]

    ARM_REST = [0.0,   # J1 ← hand X
                0.3,   # J2 ← hand Y
               -1.5,   # J3 fixed elbow-up
               -1.5,   # J4 fixed
                0.0,   # J5 fixed
                1.0,   # J6 fixed (tool points down)
                0.0]   # J7 fixed

    def __init__(self):
        super().__init__('simple_gesture_controller')

        self.arm_client     = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.grasper_client = ActionClient(self, FollowJointTrajectory, '/grasper_controller/follow_joint_trajectory')
        self.shaft_client   = ActionClient(self, FollowJointTrajectory, '/shaft_controller/follow_joint_trajectory')

        # smoothed joint state — these are the values actually sent to the controllers
        self.s_j1     = 0.0
        self.s_j2     = 0.3
        self.s_j5     = 0.0
        self.s_j6     = 1.0
        self.s_jaw    = 0.8   # starts open
        self.s_shaft  = 0.0   # starts retracted
        self.s_roll   = 0.0
        self.s_pitch  = 0.0
        self.s_yaw_joint = 0.0
        self.jaw_locked = False  # latches jaw closed until released
        self.pinch_dist = 1.0   # default "open" distance
        self.raw_dist   = 0.1   # default hand distance

        # MediaPipe hand tracker
        self.mp_hands = mp.solutions.hands
        self.hands    = self.mp_hands.Hands(
            static_image_mode=False, max_num_hands=1,
            min_detection_confidence=0.6, min_tracking_confidence=0.5)
        self.mp_draw  = mp.solutions.drawing_utils

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Camera 0 not found!')

        self.last_cmd = self.get_clock().now()
        self.create_timer(1.0 / 30.0, self.loop)

        self.get_logger().info('Simple Gesture Controller READY')
        self.get_logger().info('  ARM MODE:   Move hand L/R/U/D (Base/Shoulder)')
        self.get_logger().info('  GRASPER (☝): Pitch/Yaw (X/Y) + Roll (Hand Move)')
        self.get_logger().info('  SHAFT (✌):  Slide trocar (Depth)')
        self.get_logger().info('  WRIST (🤟):  Joint 5/6 (X/Y)')
        self.get_logger().info('  FIST:       CLOSE Jaw + LOCK ARM')

    # helpers and geometry

    def get_palm_state(self, m):
        """Calculate palm orientation (roll/pitch/yaw) and spread."""
        # build two hand vectors from wrist to index-MCP and wrist to pinky-MCP
        v1 = np.array([m[5].x - m[0].x, m[5].y - m[0].y, m[5].z - m[0].z])
        v2 = np.array([m[17].x - m[0].x, m[17].y - m[0].y, m[17].z - m[0].z])
        
        # normalise them
        v1_n = v1 / (np.linalg.norm(v1) + 1e-6)
        v2_n = v2 / (np.linalg.norm(v2) + 1e-6)
        
        # cross product gives us the palm normal direction
        norm = np.cross(v1_n, v2_n)
        norm /= (np.linalg.norm(norm) + 1e-6)
        
        pitch = float(np.arctan2(norm[1], norm[2] + 1e-6))
        yaw   = float(np.arctan2(norm[0], norm[2] + 1e-6))
        roll  = float(np.arctan2(v1_n[1], v1_n[0] + 1e-6))
        
        # distance between index and pinky tips — rough measure of how open the hand is
        spread = np.sqrt((m[8].x - m[20].x)**2 + (m[8].y - m[20].y)**2)
        
        return roll, pitch, yaw, spread

    def classify(self, m):
        """Return gesture string: 'FIST', 'OPEN', 'POINT', or 'OTHER'."""
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        ext  = [m[t].y < m[p].y for t, p in zip(tips, pips)]
        # thumb extended check
        thumb_ext = abs(m[4].x - m[2].x) > 0.06

        n_ext = sum(ext)

        # ok sign = thumb and index almost touching
        self.pinch_dist = np.sqrt((m[4].x - m[8].x)**2 + (m[4].y - m[8].y)**2 + (m[4].z - m[8].z)**2)
        if self.pinch_dist < 0.05:
            return 'OK'
            
        # thumbs up = thumb extended, all others curled
        if thumb_ext and n_ext == 0:
            return 'THUMBS_UP'

        if n_ext == 0 and not thumb_ext:
            return 'FIST'
        if n_ext >= 4:
            return 'OPEN'
        # POINT: index up, middle still down
        if ext[0] and not ext[1]:
            return 'POINT'
        # VICTORY: index and middle up, ring still down
        if ext[0] and ext[1] and not ext[2]:
            return 'VICTORY'
        # ROCK ON: thumb + index + pinky out
        if ext[0] and ext[3] and thumb_ext:
            return 'ROCK_ON'
        return 'OTHER'

    def is_thumbs_down_gesture(self, marks):
        """Detect thumb pointing down and fingers retracted."""
        thumb_down = marks[4].y > marks[3].y and marks[3].y > marks[2].y
        tips = [12, 16, 20]
        pips = [10, 14, 18]
        ext = [marks[t].y < marks[p].y for t, p in zip(tips, pips)]
        return thumb_down and sum(ext) == 0

    def get_vertical_shaft_pos(self, marks):
        """Map vertical position (Wrist Y) to shaft extension (0.0 to 0.08)."""
        wrist_y = marks[0].y
        # 0.2 (top) -> 0.08 extension, 0.7 (bottom) -> 0.0 extension
        ratio = (0.7 - wrist_y) / (0.7 - 0.2)
        return float(np.clip(ratio * 0.08, 0.0, 0.08))

    def ema(self, prev, target):
        return ALPHA * target + (1.0 - ALPHA) * prev

    def make_goal(self, names, positions):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions  = [float(p) for p in positions]
        pt.velocities = [0.0] * len(positions)
        pt.time_from_start = Duration(sec=0, nanosec=int(TRAJ_MS * 1e6))
        g.trajectory.points.append(pt)
        return g

    # main perception + control loop

    def loop(self):
        if not rclpy.ok():
            return
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res   = self.hands.process(rgb)

        h, w, _ = frame.shape

        if res.multi_hand_landmarks:
            for hand in res.multi_hand_landmarks:
                m = hand.landmark

                # wrist-to-index-MCP distance gives a rough sense of hand scale
                dx = m[0].x - m[5].x
                dy = m[0].y - m[5].y
                self.raw_dist = float(np.sqrt(dx*dx + dy*dy))

                # classify the current gesture
                gesture = self.classify(m)
                
                # reset targets to current smoothed values each loop
                jaw_target   = self.s_jaw
                roll_target  = self.s_roll
                pitch_target = self.s_pitch
                yaw_target   = self.s_yaw_joint
                shaft_target = self.s_shaft
                
                # arm starts locked; individual modes unlock what they need
                t_arm = list(self.ARM_REST)
                lock_arm   = True
                lock_shaft = True
                lock_grasp = True

                # gesture dispatch — each mode unlocks only the axes it controls
                
                # 1. JAW control — highest priority, locks everything else out
                if gesture == 'FIST' or gesture == 'OPEN':
                    if gesture == 'FIST':
                        self.jaw_locked = True
                        jaw_target = 0.0
                        label = "JAW: CLOSE [HOLD]"
                        color = (0, 0, 255)
                    else:
                        self.jaw_locked = False
                        jaw_target = 0.8
                        label = "JAW: OPEN"
                        color = (0, 255, 0)
                    # other subsystems stay locked at their current values

                # 2. SHAFT MODE via victory gesture
                elif gesture == 'VICTORY':
                    lock_shaft = False
                    shaft_target = self.get_vertical_shaft_pos(m)
                    label = "SHAFT MODE (VICTORY)"
                    color = (0, 255, 255)

                # thumbs down always overrides and retracts the shaft immediately
                if self.is_thumbs_down_gesture(m):
                    lock_shaft = False
                    shaft_target = 0.0
                    label = "EMERGENCY RETRACT!"
                    color = (0, 0, 255)

                # 3. ROTATION MODE via point gesture
                elif gesture == 'POINT':
                    lock_grasp = False
                    roll_target  = float(np.clip((m[8].x - 0.5) * 6.0, -3.14, 3.14))
                    pitch_target = float(np.clip((m[8].y - 0.5) * -3.0, -1.5, 1.5))
                    yaw_target   = float(np.clip(m[0].z * -10.0, -1.5, 1.5))
                    label = "ROTATION MODE (POINT)"
                    color = (255, 255, 0)

                # 4. WRIST MODE via rock-on gesture
                elif gesture == 'ROCK_ON':
                    lock_arm = False  # unlock J5/J6 only
                    t_arm[4] = float(np.clip((m[0].x - 0.5) * 3.0, -1.5, 1.5)) # J5
                    t_arm[5] = float(np.clip((m[0].y - 0.5) * 2.0, 0.5, 1.5)) # J6
                    label = "WRIST MODE (ROCK)"
                    color = (255, 0, 255)

                # 5. default: full 7-joint arm tracking
                else:
                    lock_arm = False
                    p_roll, p_pitch, p_yaw, p_spread = self.get_palm_state(m)
                    
                    # J1/J2: base and shoulder from wrist x/y position
                    t_arm[0] = float(np.clip((m[0].x - 0.5) * -3.0, -1.8, 1.8)) 
                    t_arm[1] = float(np.clip((m[0].y - 0.5) * -2.0, -1.2, 0.8))
                    
                    # J3/J4: elbow and mid-arm follow palm orientation
                    t_arm[2] = float(np.clip(p_yaw * 1.5, -1.5, 1.5))
                    t_arm[3] = float(np.clip(p_pitch * 1.5, -2.0, 1.5))
                    
                    # J5/J6: wrist roll from palm and height from hand scale
                    t_arm[4] = float(np.clip(p_roll * 1.5, -1.5, 1.5))
                    t_arm[5] = float(np.clip((self.raw_dist - 0.1) * 8.0, 0.0, 2.5))
                    
                    # J7: flange rotation from finger spread
                    t_arm[6] = float(np.clip((p_spread - 0.1) * 4.0, -1.5, 1.5))
                    
                    label = "FULL ARM MODE (7-DOF)"
                    color = (200, 200, 200)

                # if the jaw is locked, keep it closed regardless of gesture
                if self.jaw_locked:
                    jaw_target = 0.0
                    if "HOLD" not in label: label += " [HOLD]"

                # ── EMA Update ────────────────────────────────────────────
                if not lock_arm:
                    self.s_j1 = self.ema(self.s_j1, t_arm[0])
                    self.s_j2 = self.ema(self.s_j2, t_arm[1])
                    self.s_j5 = self.ema(self.s_j5, t_arm[4])
                    self.s_j6 = self.ema(self.s_j6, t_arm[5])
                
                if not lock_grasp:
                    self.s_roll  = self.ema(self.s_roll,  roll_target)
                    self.s_pitch = self.ema(self.s_pitch, pitch_target)
                    self.s_yaw_joint   = self.ema(self.s_yaw_joint, yaw_target)
                
                if not lock_shaft:
                    self.s_shaft = self.ema(self.s_shaft, shaft_target)

                self.s_jaw   = self.ema(self.s_jaw,   jaw_target)

                # ── Send Commands ─────────────────────────────────────────
                now = self.get_clock().now()
                if (now - self.last_cmd).nanoseconds / 1e9 >= 1.0 / CMD_HZ:
                    # build the full arm command from smoothed values
                    arm = list(self.ARM_REST)
                    arm[0], arm[1], arm[4], arm[5] = self.s_j1, self.s_j2, self.s_j5, self.s_j6
                    self._send(self.arm_client, self.ARM_JOINTS, arm)
                    
                    # grasper: roll, pitch, yaw, jaw
                    self._send(self.grasper_client, self.GRASPER_JOINTS, 
                               [self.s_roll, self.s_pitch, self.s_yaw_joint, self.s_jaw])
                    
                    # shaft depth
                    self._send(self.shaft_client, self.SHAFT_JOINT, [self.s_shaft])
                    self.last_cmd = now

                # draw hand skeleton and gesture label on the frame
                self.mp_draw.draw_landmarks(frame, hand, self.mp_hands.HAND_CONNECTIONS)
                cv2.putText(frame, label, (10, h - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

            # ── HUD ───────────────────────────────────────────────────────
            hud = [
                f'MODE    : {label}',
                f'Base/Sh : {np.degrees(self.s_j1):+.0f}/{np.degrees(self.s_j2):+.0f}',
                f'Grasper : P:{np.degrees(self.s_pitch):.0f} Y:{np.degrees(self.s_yaw_joint):.0f} R:{np.degrees(self.s_roll):.1f}',
                f'Jaw/Shaf: {self.s_jaw:.1f} / {self.s_shaft*100:.1f}cm',
                f'Diag    : Dist:{getattr(self, "raw_dist", 0.0):.3f}',
            ]
            for i, txt in enumerate(hud):
                cv2.putText(frame, txt, (10, 28 + i * 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 160), 2 if i==0 else 1)

            # quick gesture cheat-sheet in the top-right corner
            guide = ['FIST=Grasp', 'OPEN=Rel.', 'VICTORY=Shaft', 'POINT=Rot']
            for i, g in enumerate(guide):
                cv2.putText(frame, g, (w - 180, 28 + i * 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        else:
            cv2.putText(frame, 'SHOW HAND TO CAMERA', (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 50, 255), 2)

        cv2.imshow('Surgical Gesture Control', frame)
        cv2.waitKey(1)

    def _send(self, client, names, positions):
        if client.server_is_ready():
            client.send_goal_async(self.make_goal(names, positions))

    def shutdown_cleanly(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleGestureController()
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
