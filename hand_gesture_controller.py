#!/usr/bin/env python3
"""
Hand Gesture Controller — Laparoscopic Grasper Surgical Robot
==============================================================
Compatible with mediapipe >= 0.10  (Tasks API, no mp.solutions)

On first run, downloads hand_landmarker.task (~9 MB) to ~/hand_landmarker.task
Subsequent runs use the cached file.

GESTURE MAP
───────────────────────────────────────────────────────────────
  ✋ Open hand (5 fingers)        → Open grasper jaws
  ✊ Fist (0 fingers)             → Close / grasp
  ☝  Index only, pointing up     → Shoulder UP
  👇 Index only, pointing down   → Shoulder DOWN
  👈 Index only, pointing left   → Yaw LEFT
  👉 Index only, pointing right  → Yaw RIGHT
  ✌  Peace (index + middle)      → Extend elbow forward
  🤘 Three fingers (i+m+r)       → Retract elbow
  👍 Thumb only                  → Wrist roll +
  🤙 Thumb + pinky               → HOME (safe reset)

USAGE
─────
  source ~/gesture_venv/bin/activate
  python3 hand_gesture_controller.py

  Press Q or ESC to quit.
  Press H to home the robot at any time.
"""

import os
import sys
import time
import subprocess
import urllib.request
from collections import deque

import cv2
import numpy as np

# ── mediapipe Tasks API ───────────────────────────────────────────────────────
try:
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions
    from mediapipe.tasks.python.vision import (
        HandLandmarker,
        HandLandmarkerOptions,
        RunningMode,
    )
    MP_OK = True
except ImportError as e:
    print(f"[ERROR] mediapipe import failed: {e}")
    print("  Run:  pip install mediapipe opencv-python")
    sys.exit(1)

# ── ROS2 (optional) ───────────────────────────────────────────────────────────
try:
    import rclpy  # noqa: F401
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ═══════════════════════════════════════════════════════════════════════════════

MODEL_PATH      = os.path.expanduser("~/hand_landmarker.task")
MODEL_URL       = ("https://storage.googleapis.com/mediapipe-models/"
                   "hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task")

CAMERA_INDEX     = 0
GESTURE_COOLDOWN = 0.6    # seconds between repeated commands
ARM_STEP         = 0.08   # radians per gesture pulse
MOVE_DURATION    = 1      # seconds for each trajectory

JAW_OPEN  = [0.0,  0.50, -0.50,  0.30, -0.30]
JAW_CLOSE = [0.0,  0.05, -0.05,  0.04, -0.04]
HOME_ARM  = [0.0,  0.0,   0.0,   0.0]

LIMITS = {
    "joint1_yaw":      (-1.57, 1.57),
    "joint2_shoulder": (-1.0,  1.0),
    "joint3_elbow":    (-1.2,  1.2),
    "joint4_wrist":    (-3.14, 3.14),
}

C_GREEN  = (50,  220,  80)
C_BLUE   = (255, 160,  30)
C_RED    = (50,   50, 230)
C_YELLOW = (20,  220, 230)
C_WHITE  = (240, 240, 240)
C_TEAL   = (200, 180,  30)
C_GREY   = (120, 120, 120)


# ═══════════════════════════════════════════════════════════════════════════════
#  MODEL DOWNLOAD
# ═══════════════════════════════════════════════════════════════════════════════

def ensure_model() -> str:
    if os.path.exists(MODEL_PATH):
        print(f"[INFO] Using cached model: {MODEL_PATH}")
        return MODEL_PATH

    print(f"[INFO] Downloading hand landmark model (~9 MB)...")
    print(f"       {MODEL_URL}")

    def _progress(block_num, block_size, total_size):
        downloaded = block_num * block_size
        if total_size > 0:
            pct = min(downloaded / total_size * 100, 100)
            bar = int(pct / 2)
            print(f"\r  [{'#'*bar}{'.'*(50-bar)}] {pct:.0f}%", end="", flush=True)

    try:
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH, _progress)
        print("\n[INFO] Model downloaded successfully.")
        return MODEL_PATH
    except Exception as e:
        print(f"\n[ERROR] Download failed: {e}")
        print("\n  Manual download command:")
        print(f"  wget -O ~/hand_landmarker.task \\")
        print(f"    '{MODEL_URL}'")
        sys.exit(1)


# ═══════════════════════════════════════════════════════════════════════════════
#  ROBOT COMMANDER
# ═══════════════════════════════════════════════════════════════════════════════

class RobotCommander:
    ARM_JOINTS = ["joint1_yaw", "joint2_shoulder", "joint3_elbow", "joint4_wrist"]
    GRASP_JOINTS = [
        "joint5_wrist_pitch",
        "finger_left_proximal_joint", "finger_right_proximal_joint",
        "finger_left_distal_joint",   "finger_right_distal_joint",
    ]

    def _fire(self, controller, joints, positions, duration):
        if not ROS2_AVAILABLE:
            return
        pos_str = ", ".join(f"{p:.4f}" for p in positions)
        vel_str = ", ".join("0.0" for _ in positions)
        cmd = (
            f'ros2 action send_goal /{controller}/follow_joint_trajectory '
            f'control_msgs/action/FollowJointTrajectory "{{'
            f'trajectory: {{joint_names: {joints}, '
            f'points: [{{positions: [{pos_str}], velocities: [{vel_str}], '
            f'time_from_start: {{sec: {int(duration)}, nanosec: 0}}}}]}}}}"'
        )
        subprocess.Popen(cmd, shell=True,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def send_arm(self, positions, duration=MOVE_DURATION):
        self._fire("arm_controller", self.ARM_JOINTS, positions, duration)

    def send_grasper(self, positions, duration=MOVE_DURATION):
        self._fire("grasper_controller", self.GRASP_JOINTS, positions, duration)


# ═══════════════════════════════════════════════════════════════════════════════
#  ARM STATE
# ═══════════════════════════════════════════════════════════════════════════════

class ArmState:
    def __init__(self):
        self.yaw = self.shoulder = self.elbow = self.wrist = 0.0

    def clamp(self):
        def cl(v, k):
            lo, hi = LIMITS[k]
            return max(lo, min(hi, v))
        self.yaw      = cl(self.yaw,      "joint1_yaw")
        self.shoulder = cl(self.shoulder, "joint2_shoulder")
        self.elbow    = cl(self.elbow,    "joint3_elbow")
        self.wrist    = cl(self.wrist,    "joint4_wrist")

    def as_list(self):
        return [self.yaw, self.shoulder, self.elbow, self.wrist]

    def reset(self):
        self.yaw = self.shoulder = self.elbow = self.wrist = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
#  GESTURE DETECTOR  (mediapipe Tasks API)
# ═══════════════════════════════════════════════════════════════════════════════

TIP = [4, 8, 12, 16, 20]
PIP = [3, 6, 10, 14, 18]

HAND_CONNECTIONS = [
    (0,1),(1,2),(2,3),(3,4),
    (0,5),(5,6),(6,7),(7,8),
    (0,9),(9,10),(10,11),(11,12),
    (0,13),(13,14),(14,15),(15,16),
    (0,17),(17,18),(18,19),(19,20),
    (5,9),(9,13),(13,17),
]


class GestureDetector:

    def __init__(self, model_path: str):
        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.IMAGE,
            num_hands=1,
            min_hand_detection_confidence=0.65,
            min_hand_presence_confidence=0.65,
            min_tracking_confidence=0.55,
        )
        self.landmarker = HandLandmarker.create_from_options(options)

    def _fingers_up(self, lm, is_right: bool) -> list:
        up = []
        # Thumb: horizontal comparison
        if is_right:
            up.append(1 if lm[TIP[0]].x < lm[PIP[0]].x else 0)
        else:
            up.append(1 if lm[TIP[0]].x > lm[PIP[0]].x else 0)
        # Other fingers: tip y above pip y
        for i in range(1, 5):
            up.append(1 if lm[TIP[i]].y < lm[PIP[i]].y else 0)
        return up

    def _classify(self, fingers, lm):
        t, i, m, r, p = fingers

        if sum(fingers) == 5:
            return "OPEN_HAND", 0.95

        if sum(fingers) == 0:
            return "FIST", 0.95

        if t == 1 and i == 0 and m == 0 and r == 0 and p == 1:
            return "HANG_LOOSE", 0.92

        if t == 1 and i == 0 and m == 0 and r == 0 and p == 0:
            return "THUMB_UP", 0.88

        # 1 finger — forward/back (Z depth) or up/down/left/right
        if t == 0 and i == 1 and m == 0 and r == 0 and p == 0:
            dx = lm[8].x - lm[0].x
            dy = lm[8].y - lm[0].y
            dz = lm[8].z - lm[5].z   # negative = tip closer to camera
            # Z gesture wins if tip is clearly coming toward/away camera
            # and not strongly pointing up/down/sideways
            if abs(dz) > 0.04 and abs(dy) < 0.25 and abs(dx) < 0.30:
                return ("PUSH_FORWARD" if dz > 0 else "PUSH_BACK"), 0.90
            # Otherwise directional
            if abs(dy) > abs(dx) * 1.1:
                return ("ARM_UP" if dy < 0 else "ARM_DOWN"), 0.88
            else:
                return ("YAW_RIGHT" if dx > 0 else "YAW_LEFT"), 0.85

        # 2 fingers — forward/back (Z depth) or lateral extend
        if t == 0 and i == 1 and m == 1 and r == 0 and p == 0:
            dz_i = lm[8].z  - lm[5].z
            dz_m = lm[12].z - lm[9].z
            dz   = (dz_i + dz_m) / 2
            if abs(dz) > 0.04:
                return ("TWO_FORWARD" if dz > 0 else "TWO_BACK"), 0.90
            return "EXTEND", 0.85

        if t == 0 and i == 1 and m == 1 and r == 1 and p == 0:
            return "RETRACT", 0.85

        return "UNKNOWN", 0.0

    def process(self, bgr_frame):
        h, w = bgr_frame.shape[:2]
        rgb    = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        result     = self.landmarker.detect(mp_img)
        annotated  = bgr_frame.copy()

        if not result.hand_landmarks:
            return "NONE", 0.0, annotated

        lm   = result.hand_landmarks[0]
        pts  = [(int(p.x * w), int(p.y * h)) for p in lm]

        # Draw skeleton
        for a, b in HAND_CONNECTIONS:
            cv2.line(annotated, pts[a], pts[b], (60, 200, 60), 2)
        for x, y in pts:
            cv2.circle(annotated, (x, y), 5, (255, 255, 255), -1)
            cv2.circle(annotated, (x, y), 5, (30, 160, 30), 2)

        is_right = True
        if result.handedness:
            is_right = result.handedness[0][0].category_name == "Right"

        fingers = self._fingers_up(lm, is_right)
        name, conf = self._classify(fingers, lm)
        return name, conf, annotated


# ═══════════════════════════════════════════════════════════════════════════════
#  OVERLAY
# ═══════════════════════════════════════════════════════════════════════════════

GESTURE_INFO = {
    "OPEN_HAND":  ("Open Hand",    "Open grasper jaws",   C_GREEN),
    "FIST":       ("Fist",         "Close / Grasp",       C_RED),
    "ARM_UP":     ("Point Up",     "Shoulder UP",         C_YELLOW),
    "ARM_DOWN":   ("Point Down",   "Shoulder DOWN",       C_YELLOW),
    "YAW_LEFT":   ("Point Left",   "Yaw LEFT",            C_TEAL),
    "YAW_RIGHT":  ("Point Right",  "Yaw RIGHT",           C_TEAL),
    "HANG_LOOSE": ("Hang Loose",   "HOME position",       C_BLUE),
    "EXTEND":     ("Peace V",      "Extend elbow +",      C_WHITE),
    "RETRACT":    ("Three Fingers","Retract elbow -",     C_WHITE),
    "THUMB_UP":     ("Thumb Up",       "Wrist roll +",         (160, 255, 160)),
    "PUSH_FORWARD": ("1 Finger FWD",  "Elbow FORWARD",        C_GREEN),
    "PUSH_BACK":    ("1 Finger BACK", "Elbow BACK",           C_RED),
    "TWO_FORWARD":  ("2 Fingers FWD", "Shoulder FORWARD",     (100, 255, 180)),
    "TWO_BACK":     ("2 Fingers BACK","Shoulder BACK",        (100, 100, 255)),
    "NONE":         ("No hand",       "---",                  C_GREY),
    "UNKNOWN":      ("Unknown",       "No action",            C_GREY),
}


def draw_overlay(frame, gesture, conf, arm, last_cmd, fps, cd_pct):
    h, w = frame.shape[:2]
    out  = frame.copy()

    # Top bar
    cv2.rectangle(out, (0, 0), (w, 50), (15, 15, 15), -1)
    cv2.putText(out, "Surgical Robot | Hand Gesture Control",
                (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.70, C_BLUE, 2)
    cv2.putText(out, f"FPS {fps:.0f}",
                (w - 85, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.62, C_GREEN, 2)

    # Gesture bar (bottom)
    label, desc, colour = GESTURE_INFO.get(gesture, GESTURE_INFO["UNKNOWN"])
    cv2.rectangle(out, (0, h - 80), (w, h), (15, 15, 15), -1)
    cv2.putText(out, label, (12, h - 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, colour, 2)
    cv2.putText(out, desc,  (12, h - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.60, C_WHITE, 1)
    bar_w = int((w - 24) * conf)
    cv2.rectangle(out, (12, h - 7), (12 + bar_w, h - 2), colour, -1)

    # Cooldown arc (top-right)
    cx, cy, r = w - 45, 85, 24
    cv2.circle(out, (cx, cy), r, (50, 50, 50), 3)
    if cd_pct < 1.0:
        cv2.ellipse(out, (cx, cy), (r, r), -90, 0, int(360 * cd_pct), C_GREEN, 3)

    # Joint state panel
    px = w - 210
    cv2.rectangle(out, (px - 6, 58), (w - 4, 222), (20, 20, 20), -1)
    cv2.rectangle(out, (px - 6, 58), (w - 4, 222), (60, 60, 60), 1)
    cv2.putText(out, "ARM STATE", (px, 78),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, C_BLUE, 1)
    jv = [("Yaw",      arm.yaw,      LIMITS["joint1_yaw"]),
          ("Shoulder", arm.shoulder, LIMITS["joint2_shoulder"]),
          ("Elbow",    arm.elbow,    LIMITS["joint3_elbow"]),
          ("Wrist",    arm.wrist,    LIMITS["joint4_wrist"])]
    for idx, (nm, val, (lo, hi)) in enumerate(jv):
        y = 100 + idx * 28
        cv2.putText(out, f"{nm:<9} {val:+.2f}",
                    (px, y), cv2.FONT_HERSHEY_SIMPLEX, 0.43, C_WHITE, 1)
        bw = int(120 * (val - lo) / (hi - lo))
        cv2.rectangle(out, (px, y + 4), (px + 120, y + 10), (60, 60, 60), -1)
        cv2.rectangle(out, (px, y + 4), (px + bw,  y + 10), C_TEAL, -1)

    if last_cmd:
        cv2.putText(out, f">> {last_cmd}", (px, 212),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, C_GREEN, 1)

    # Legend
    lines = ["Open=jaws open",  "Fist=grasp",
             "1fin up/dn",      "1fin L/R=yaw",
             "1fin FWD=elbow+", "1fin BACK=elbow-",
             "2fin FWD=shldr+", "2fin BACK=shldr-",
             "V=elbow fwd",     "Loose=HOME"]
    cv2.rectangle(out, (6, 56), (185, 56 + len(lines) * 17 + 4), (20, 20, 20), -1)
    for i, txt in enumerate(lines):
        cv2.putText(out, txt, (10, 70 + i * 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (160, 160, 160), 1)

    ros_col = C_GREEN if ROS2_AVAILABLE else C_RED
    cv2.putText(out, "ROS2 LIVE" if ROS2_AVAILABLE else "DEMO MODE",
                (10, h - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.48, ros_col, 1)

    return out


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    print("\n" + "=" * 60)
    print("  Laparoscopic Grasper — Hand Gesture Controller")
    print("  mediapipe Tasks API | OpenCV | ROS2 Jazzy")
    print("=" * 60)
    print(f"  ROS2 available : {ROS2_AVAILABLE}")

    model_path = ensure_model()
    print(f"  Model          : {model_path}")
    print(f"  Camera index   : {CAMERA_INDEX}")
    print("  Press Q / ESC to quit  |  H = home robot")
    print("=" * 60 + "\n")

    detector  = GestureDetector(model_path)
    commander = RobotCommander()
    arm       = ArmState()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {CAMERA_INDEX}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    last_cmd_time = 0.0
    last_gesture  = ""
    last_cmd      = ""
    fps_buf       = deque(maxlen=20)
    t_prev        = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.03)
            continue

        frame = cv2.flip(frame, 1)

        gesture, conf, annotated = detector.process(frame)

        now     = time.time()
        fps_buf.append(1.0 / max(now - t_prev, 1e-6))
        t_prev  = now
        fps     = float(np.mean(fps_buf))
        elapsed = now - last_cmd_time
        cd_pct  = min(elapsed / GESTURE_COOLDOWN, 1.0)

        # ── Dispatch ─────────────────────────────────────────────────────
        if (gesture not in ("NONE", "UNKNOWN") and
                conf >= 0.80 and
                elapsed >= GESTURE_COOLDOWN and
                gesture != last_gesture):

            last_cmd_time = now
            last_gesture  = gesture
            acted         = True

            if gesture == "OPEN_HAND":
                commander.send_grasper(JAW_OPEN)
                last_cmd = "Grasper OPEN"

            elif gesture == "FIST":
                commander.send_grasper(JAW_CLOSE)
                last_cmd = "Grasper CLOSE"

            elif gesture == "ARM_UP":
                arm.shoulder += ARM_STEP; arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd = f"Shoulder UP  {arm.shoulder:+.2f}"

            elif gesture == "ARM_DOWN":
                arm.shoulder -= ARM_STEP; arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd = f"Shoulder DN  {arm.shoulder:+.2f}"

            elif gesture == "YAW_LEFT":
                arm.yaw -= ARM_STEP; arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd = f"Yaw LEFT     {arm.yaw:+.2f}"

            elif gesture == "YAW_RIGHT":
                arm.yaw += ARM_STEP; arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd = f"Yaw RIGHT    {arm.yaw:+.2f}"

            elif gesture == "EXTEND":
                arm.elbow += ARM_STEP; arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd = f"Elbow EXT    {arm.elbow:+.2f}"

            elif gesture == "RETRACT":
                arm.elbow -= ARM_STEP; arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd = f"Elbow RET    {arm.elbow:+.2f}"

            elif gesture == "THUMB_UP":
                arm.wrist += ARM_STEP; arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd = f"Wrist +      {arm.wrist:+.2f}"

            elif gesture == "PUSH_FORWARD":
                arm.elbow += ARM_STEP; arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd = f"1-Fwd ELBOW+ {arm.elbow:+.2f}"

            elif gesture == "PUSH_BACK":
                arm.elbow -= ARM_STEP; arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd = f"1-Back ELBOW-{arm.elbow:+.2f}"

            elif gesture == "TWO_FORWARD":
                arm.shoulder += ARM_STEP; arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd = f"2-Fwd SHLDR+ {arm.shoulder:+.2f}"

            elif gesture == "TWO_BACK":
                arm.shoulder -= ARM_STEP; arm.clamp()
                commander.send_arm(arm.as_list())
                last_cmd = f"2-Back SHLDR-{arm.shoulder:+.2f}"

            elif gesture == "HANG_LOOSE":
                arm.reset()
                commander.send_arm(HOME_ARM, duration=2.5)
                commander.send_grasper(JAW_OPEN, duration=2.0)
                last_cmd = "HOME"

            else:
                acted = False

            if acted:
                print(f"[GESTURE] {gesture:<14} -> {last_cmd}")

        elif elapsed >= GESTURE_COOLDOWN:
            last_gesture = ""

        display = draw_overlay(annotated, gesture, conf, arm, last_cmd, fps, cd_pct)
        cv2.imshow("Surgical Robot | Gesture Control", display)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("h"):
            arm.reset()
            commander.send_arm(HOME_ARM, duration=2.5)
            last_cmd = "HOME (hotkey)"

    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Gesture controller stopped.")


if __name__ == "__main__":
    main()
