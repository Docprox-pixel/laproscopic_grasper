import cv2
import mediapipe as mp
import numpy as np
import subprocess
import threading
import time
import sys
from collections import deque
from dataclasses import dataclass
from typing import Optional

# ── ROS2 optional import ─────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[WARN] ROS2 not available — running in DEMO mode (overlay only, no commands sent)")


CAMERA_INDEX      = 0        # webcam index
GESTURE_COOLDOWN  = 0.6      # seconds between repeated same-gesture commands
ARM_STEP          = 0.08     # radians per gesture pulse
JAW_OPEN          = [0.0,  0.50, -0.50,  0.30, -0.30]   # [wp, lp, rp, ld, rd]
JAW_CLOSE         = [0.0,  0.05, -0.05,  0.04, -0.04]
JAW_HALF          = [0.0,  0.25, -0.25,  0.15, -0.15]
HOME_ARM          = [0.0, 0.0, 0.0, 0.0]                 # [yaw, shoulder, elbow, wrist]
MOVE_DURATION     = 1.5      # seconds for each trajectory step

# Joint limits (radians)
LIMITS = {
    "joint1_yaw":      (-1.57, 1.57),
    "joint2_shoulder": (-1.0,  1.0),
    "joint3_elbow":    (-1.2,  1.2),
    "joint4_wrist":    (-3.14, 3.14),
}

# Colours (BGR)
C_GREEN  = (50,  220,  80)
C_BLUE   = (255, 160,  30)
C_RED    = (50,   50, 230)
C_YELLOW = (20,  220, 230)
C_WHITE  = (240, 240, 240)
C_DARK   = (20,   20,  20)
C_TEAL   = (200, 180,  30)

class RobotCommander:
    """Thin wrapper that sends FollowJointTrajectory goals via subprocess
    (works even when this script isn't a proper ROS node)."""

    ARM_JOINTS = ["joint1_yaw", "joint2_shoulder", "joint3_elbow", "joint4_wrist"]
    GRASP_JOINTS = [
        "joint5_wrist_pitch",
        "finger_left_proximal_joint", "finger_right_proximal_joint",
        "finger_left_distal_joint",   "finger_right_distal_joint",
    ]

    def __init__(self):
        self._lock    = threading.Lock()
        self._busy    = False

    def _ros2_action(self, controller: str, joint_names: list, positions: list, duration: float):
        """Fire-and-forget subprocess call to ros2 action send_goal."""
        pos_str  = ", ".join(f"{p:.4f}" for p in positions)
        vel_str  = ", ".join("0.0" for _ in positions)
        dur_sec  = int(duration)

        cmd = (
            f'ros2 action send_goal /{controller}/follow_joint_trajectory '
            f'control_msgs/action/FollowJointTrajectory "{{'
            f'trajectory: {{joint_names: {joint_names}, '
            f'points: [{{positions: [{pos_str}], velocities: [{vel_str}], '
            f'time_from_start: {{sec: {dur_sec}, nanosec: 0}}}}]}}}}"'
        )
        subprocess.Popen(cmd, shell=True,
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)

    def send_arm(self, positions: list, duration: float = MOVE_DURATION):
        if not ROS2_AVAILABLE:
            return
        with self._lock:
            self._ros2_action("arm_controller", self.ARM_JOINTS, positions, duration)

    def send_grasper(self, positions: list, duration: float = MOVE_DURATION):
        if not ROS2_AVAILABLE:
            return
        with self._lock:
            self._ros2_action("grasper_controller", self.GRASP_JOINTS, positions, duration)

class ArmState:
    def __init__(self):
        self.yaw      = 0.0
        self.shoulder = 0.0
        self.elbow    = 0.0
        self.wrist    = 0.0

    def clamp(self):
        def cl(v, name): 
            lo, hi = LIMITS[name]
            return max(lo, min(hi, v))
        self.yaw      = cl(self.yaw,      "joint1_yaw")
        self.shoulder = cl(self.shoulder, "joint2_shoulder")
        self.elbow    = cl(self.elbow,    "joint3_elbow")
        self.wrist    = cl(self.wrist,    "joint4_wrist")

    def as_list(self):
        return [self.yaw, self.shoulder, self.elbow, self.wrist]

    def reset(self):
        self.yaw = self.shoulder = self.elbow = self.wrist = 0.0

@dataclass
class GestureResult:
    name:        str
    confidence:  float   # 0-1
    landmarks:   object  # mediapipe landmarks


class GestureDetector:

    # Finger tip / pip landmark indices
    TIPS = [4, 8, 12, 16, 20]   # thumb, index, middle, ring, pinky
    PIPS = [3, 6, 10, 14, 18]

    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands    = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.70,
            min_tracking_confidence=0.60,
        )
        self.mp_draw  = mp.solutions.drawing_utils
        self.mp_style = mp.solutions.drawing_styles

    def _fingers_up(self, lm, handedness: str) -> list:
        """Return [thumb, index, middle, ring, pinky] 1=up 0=down."""
        up = []
        # Thumb: compare x (flipped for right hand)
        if handedness == "Right":
            up.append(1 if lm[4].x < lm[3].x else 0)
        else:
            up.append(1 if lm[4].x > lm[3].x else 0)
        # Other fingers: tip y < pip y means extended
        for tip, pip in zip(self.TIPS[1:], self.PIPS[1:]):
            up.append(1 if lm[tip].y < lm[pip].y else 0)
        return up

    def _classify(self, fingers: list, lm, handedness: str) -> tuple[str, float]:
        """Map finger pattern → gesture name + confidence."""
        t, i, m, r, p = fingers

        # ── Open hand ──
        if sum(fingers) == 5:
            return "OPEN_HAND", 0.95

        # ── Fist ──
        if sum(fingers) == 0:
            return "FIST", 0.95

        # ── Thumbs up / Hang-loose ──
        if t == 1 and i == 0 and m == 0 and r == 0 and p == 1:
            return "HANG_LOOSE", 0.90   # 🤙  HOME

        # ── Point index only ──
        if t == 0 and i == 1 and m == 0 and r == 0 and p == 0:
            # Determine direction from wrist→index tip vector
            dx = lm[8].x - lm[0].x
            dy = lm[8].y - lm[0].y
            if abs(dy) > abs(dx) * 1.2:
                return ("ARM_UP" if dy < 0 else "ARM_DOWN"), 0.88
            else:
                return ("YAW_RIGHT" if dx > 0 else "YAW_LEFT"), 0.85

        # ── Peace / V ──
        if t == 0 and i == 1 and m == 1 and r == 0 and p == 0:
            return "EXTEND", 0.88

        # ── Three fingers (index+middle+ring) ──
        if t == 0 and i == 1 and m == 1 and r == 1 and p == 0:
            return "RETRACT", 0.85

        # ── Thumbs up ──
        if t == 1 and i == 0 and m == 0 and r == 0 and p == 0:
            return "THUMB_UP", 0.88   # wrist roll +

        # ── Pinky only ──
        if t == 0 and i == 0 and m == 0 and r == 0 and p == 1:
            return "PINKY", 0.80      # wrist roll -

        return "UNKNOWN", 0.0

    def process(self, bgr_frame) -> tuple[Optional[GestureResult], np.ndarray]:
        rgb    = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)
        annotated = bgr_frame.copy()

        if not result.multi_hand_landmarks:
            return None, annotated

        lm_obj     = result.multi_hand_landmarks[0]
        handedness = result.multi_handedness[0].classification[0].label

        # Draw skeleton
        self.mp_draw.draw_landmarks(
            annotated, lm_obj,
            self.mp_hands.HAND_CONNECTIONS,
            self.mp_style.get_default_hand_landmarks_style(),
            self.mp_style.get_default_hand_connections_style(),
        )

        lm = lm_obj.landmark
        fingers = self._fingers_up(lm, handedness)
        name, conf = self._classify(fingers, lm, handedness)

        return GestureResult(name=name, confidence=conf, landmarks=lm), annotated

GESTURE_INFO = {
    "OPEN_HAND":  ("✋ Open Hand",    "Open grasper jaws",    C_GREEN),
    "FIST":       ("✊ Fist",          "Close / Grasp",        C_RED),
    "ARM_UP":     ("☝  Point Up",     "Shoulder UP",          C_YELLOW),
    "ARM_DOWN":   ("👇 Point Down",   "Shoulder DOWN",        C_YELLOW),
    "YAW_LEFT":   ("👈 Point Left",   "Yaw LEFT",             C_TEAL),
    "YAW_RIGHT":  ("👉 Point Right",  "Yaw RIGHT",            C_TEAL),
    "HANG_LOOSE": ("🤙 Hang Loose",   "HOME position",        C_BLUE),
    "EXTEND":     ("✌  Peace",        "Extend arm (elbow+)",  C_WHITE),
    "RETRACT":    ("🤘 Three Fingers", "Retract arm (elbow-)", C_WHITE),
    "THUMB_UP":   ("👍 Thumb Up",     "Wrist roll +",         (160, 255, 160)),
    "PINKY":      ("🤙 Pinky",        "Wrist roll -",         (160, 160, 255)),
    "UNKNOWN":    ("❓ Unknown",       "No action",            (120, 120, 120)),
}

def render_overlay(frame, gesture: Optional[GestureResult],
                   arm: ArmState, last_cmd: str,
                   fps: float, cooldown_pct: float) -> np.ndarray:
    h, w = frame.shape[:2]
    out  = frame.copy()

    cv2.rectangle(out, (0, 0), (w, 54), (15, 15, 15), -1)
    cv2.putText(out, "Surgical Robot  |  Hand Gesture Control",
                (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, C_BLUE, 2)
    cv2.putText(out, f"FPS {fps:.0f}",
                (w - 90, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.65, C_GREEN, 2)

    if gesture and gesture.name != "UNKNOWN":
        label, desc, colour = GESTURE_INFO.get(gesture.name, GESTURE_INFO["UNKNOWN"])
        cv2.rectangle(out, (0, h - 90), (w, h), (15, 15, 15), -1)
        cv2.putText(out, label, (12, h - 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)
        cv2.putText(out, desc,  (12, h - 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, C_WHITE, 1)
        # confidence bar
        bar_w = int((w - 24) * gesture.confidence)
        cv2.rectangle(out, (12, h - 10), (12 + bar_w, h - 4), colour, -1)
      
    cx, cy, r = w - 50, 100, 28
    cv2.circle(out, (cx, cy), r, (50, 50, 50), 3)
    angle = int(360 * cooldown_pct)
    if angle > 0:
        cv2.ellipse(out, (cx, cy), (r, r), -90, 0, angle, C_GREEN, 3)
      
    panel_x = w - 220
    cv2.rectangle(out, (panel_x - 8, 62), (w - 4, 230), (20, 20, 20), -1)
    cv2.rectangle(out, (panel_x - 8, 62), (w - 4, 230), (60, 60, 60), 1)
    cv2.putText(out, "ARM STATE", (panel_x, 84),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, C_BLUE, 1)

    joint_vals = [
        ("Yaw",      arm.yaw,      LIMITS["joint1_yaw"]),
        ("Shoulder", arm.shoulder, LIMITS["joint2_shoulder"]),
        ("Elbow",    arm.elbow,    LIMITS["joint3_elbow"]),
        ("Wrist",    arm.wrist,    LIMITS["joint4_wrist"]),
    ]
    for idx, (name, val, (lo, hi)) in enumerate(joint_vals):
        y = 108 + idx * 30
        cv2.putText(out, f"{name:<9} {val:+.2f}r",
                    (panel_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_WHITE, 1)
        # mini bar
        bar_range = hi - lo
        bar_fill  = int(130 * (val - lo) / bar_range)
        cv2.rectangle(out, (panel_x, y + 5), (panel_x + 130, y + 11), (60, 60, 60), -1)
        cv2.rectangle(out, (panel_x, y + 5), (panel_x + bar_fill, y + 11), C_TEAL, -1)

    
    if last_cmd:
        cv2.putText(out, f"Sent: {last_cmd}", (panel_x, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_GREEN, 1)

  
    legend = [
        ("✋ open jaws", "✊ grasp"),
        ("☝ up/down",   "← → yaw"),
        ("✌ extend",    "3 retract"),
        ("🤙 home",     "👍👎 wrist"),
    ]
    lx, ly = 12, 68
    cv2.rectangle(out, (lx - 4, ly - 14), (lx + 250, ly + len(legend) * 18 + 4),
                  (20, 20, 20), -1)
    for i, (a, b) in enumerate(legend):
        cv2.putText(out, f"{a:<16}  {b}",
                    (lx, ly + i * 18), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (170, 170, 170), 1)

    
    ros_txt   = "ROS2 LIVE" if ROS2_AVAILABLE else "DEMO MODE"
    ros_color = C_GREEN if ROS2_AVAILABLE else C_RED
    cv2.putText(out, ros_txt, (12, h - 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, ros_color, 1)

    return out

def main():
    print("\n" + "=" * 60)
    print("  Laparoscopic Grasper — Hand Gesture Controller")
    print("  OpenCV + MediaPipe  →  ROS2 Jazzy")
    print("=" * 60)
    print(f"  ROS2 available : {ROS2_AVAILABLE}")
    print(f"  Camera index   : {CAMERA_INDEX}")
    print(f"  Gesture cooldown: {GESTURE_COOLDOWN}s")
    print("  Press  Q  to quit")
    print("=" * 60 + "\n")

    detector  = GestureDetector()
    commander = RobotCommander()
    arm       = ArmState()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera index {CAMERA_INDEX}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    last_cmd_time : float        = 0.0
    last_gesture  : str          = ""
    last_cmd_label: str          = ""
    fps_buf                      = deque(maxlen=20)
    t_prev                       = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[WARN] Frame read failed — retrying…")
            time.sleep(0.05)
            continue

        frame = cv2.flip(frame, 1)   # mirror so left/right feel natural

        gesture, annotated = detector.process(frame)

        # FPS
        now = time.time()
        fps_buf.append(1.0 / max(now - t_prev, 1e-6))
        t_prev = now
        fps    = np.mean(fps_buf)

        # Cooldown progress 0→1 (for arc indicator)
        elapsed      = now - last_cmd_time
        cooldown_pct = min(elapsed / GESTURE_COOLDOWN, 1.0)

        if (gesture and
                gesture.name not in ("UNKNOWN", last_gesture) and
                gesture.confidence >= 0.80 and
                elapsed >= GESTURE_COOLDOWN):

            g = gesture.name
            last_cmd_time = now
            last_gesture  = g
            cmd_sent      = True

            if g == "OPEN_HAND":
                commander.send_grasper(JAW_OPEN)
                last_cmd_label = "Grasper: OPEN"

            elif g == "FIST":
                commander.send_grasper(JAW_CLOSE)
                last_cmd_label = "Grasper: CLOSE"

            elif g == "ARM_UP":
                arm.shoulder = min(arm.shoulder + ARM_STEP,
                                   LIMITS["joint2_shoulder"][1])
                arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd_label = f"Shoulder ↑ {arm.shoulder:+.2f}"

            elif g == "ARM_DOWN":
                arm.shoulder = max(arm.shoulder - ARM_STEP,
                                   LIMITS["joint2_shoulder"][0])
                arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd_label = f"Shoulder ↓ {arm.shoulder:+.2f}"

            elif g == "YAW_LEFT":
                arm.yaw = max(arm.yaw - ARM_STEP, LIMITS["joint1_yaw"][0])
                arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd_label = f"Yaw ← {arm.yaw:+.2f}"

            elif g == "YAW_RIGHT":
                arm.yaw = min(arm.yaw + ARM_STEP, LIMITS["joint1_yaw"][1])
                arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd_label = f"Yaw → {arm.yaw:+.2f}"

            elif g == "EXTEND":
                arm.elbow = min(arm.elbow + ARM_STEP, LIMITS["joint3_elbow"][1])
                arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd_label = f"Elbow extend {arm.elbow:+.2f}"

            elif g == "RETRACT":
                arm.elbow = max(arm.elbow - ARM_STEP, LIMITS["joint3_elbow"][0])
                arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd_label = f"Elbow retract {arm.elbow:+.2f}"

            elif g == "THUMB_UP":
                arm.wrist = min(arm.wrist + ARM_STEP, LIMITS["joint4_wrist"][1])
                arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd_label = f"Wrist + {arm.wrist:+.2f}"

            elif g == "PINKY":
                arm.wrist = max(arm.wrist - ARM_STEP, LIMITS["joint4_wrist"][0])
                arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd_label = f"Wrist - {arm.wrist:+.2f}"

            elif g == "HANG_LOOSE":
                arm.reset()
                commander.send_arm(HOME_ARM, duration=2.5)
                commander.send_grasper(JAW_OPEN, duration=2.0)
                last_cmd_label = "🏠 HOME"

            else:
                cmd_sent = False

            if cmd_sent:
                print(f"[GESTURE] {g:14}  →  {last_cmd_label}")

        else:
            # Allow same gesture to repeat after cooldown
            if elapsed >= GESTURE_COOLDOWN:
                last_gesture = ""

  
        display = render_overlay(
            annotated, gesture, arm,
            last_cmd_label, fps, cooldown_pct,
        )
        cv2.imshow("Surgical Robot  |  Gesture Control", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        if key == ord("h"):   # manual home hotkey
            arm.reset()
            commander.send_arm(HOME_ARM, duration=2.5)
            last_cmd_label = "🏠 HOME (hotkey)"

    cap.release()
    cv2.destroyAllWindows()
    print("\n[INFO] Gesture controller stopped.")


if __name__ == "__main__":
    main()
