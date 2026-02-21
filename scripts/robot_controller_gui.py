#!/usr/bin/env python3
"""
Surgical Robot GUI Controller
Directly sends joint commands via ros2 action client
"""

import subprocess
import threading
import tkinter as tk
from tkinter import ttk
import json

# Current joint positions
joints = {
    "joint1_yaw":      0.0,
    "joint2_shoulder": 0.0,
    "joint3_elbow":    0.0,
    "joint4_wrist":    0.0,
    "jaw_left_joint":  0.0,
    "jaw_right_joint": 0.0,
}

def send_arm_command(j1, j2, j3, j4, duration=3):
    cmd = f"""ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{{
  trajectory: {{
    joint_names: ['joint1_yaw','joint2_shoulder','joint3_elbow','joint4_wrist'],
    points: [{{
      positions: [{j1}, {j2}, {j3}, {j4}],
      velocities: [0.0, 0.0, 0.0, 0.0],
      time_from_start: {{sec: {duration}, nanosec: 0}}
    }}]
  }}
}}" """
    subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def send_jaw_command(left, right, duration=2):
    cmd = f"""ros2 action send_goal /grasper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{{
  trajectory: {{
    joint_names: ['jaw_left_joint','jaw_right_joint'],
    points: [{{
      positions: [{left}, {right}],
      velocities: [0.0, 0.0],
      time_from_start: {{sec: {duration}, nanosec: 0}}
    }}]
  }}
}}" """
    subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def run_full_sequence():
    status_var.set("Running surgical sequence...")
    def sequence():
        import time
        # Phase 1: Home
        status_var.set("Phase 1: Home position")
        send_arm_command(0.0, 0.0, 0.0, 0.0, 2)
        time.sleep(3)
        # Phase 2: Position over patient
        status_var.set("Phase 2: Positioning over patient...")
        send_arm_command(0.45, 0.5, 0.15, 0.0, 6)
        time.sleep(7)
        # Phase 3: Open jaws
        status_var.set("Phase 3: Opening jaws...")
        send_jaw_command(0.5, -0.5, 2)
        time.sleep(3)
        # Phase 4: Insert trocar
        status_var.set("Phase 4: Inserting trocar...")
        send_arm_command(0.45, 0.65, 0.35, 0.0, 5)
        time.sleep(6)
        # Phase 5: Navigate to tissue
        status_var.set("Phase 5: Navigating to tissue...")
        send_arm_command(0.35, 0.68, 0.40, 0.3, 5)
        time.sleep(6)
        # Phase 6: Grasp
        status_var.set("Phase 6: GRASPING tissue...")
        send_jaw_command(0.12, -0.12, 3)
        time.sleep(4)
        # Phase 7: Lift
        status_var.set("Phase 7: Lifting tissue...")
        send_arm_command(0.35, 0.50, 0.30, 0.3, 7)
        time.sleep(8)
        # Phase 8: Release
        status_var.set("Phase 8: Releasing tissue...")
        send_jaw_command(0.5, -0.5, 2)
        time.sleep(3)
        # Phase 9: Retract
        status_var.set("Phase 9: Retracting to home...")
        send_arm_command(0.0, 0.0, 0.0, 0.0, 9)
        send_jaw_command(0.0, 0.0, 2)
        time.sleep(10)
        status_var.set("✓ Sequence complete!")
    threading.Thread(target=sequence, daemon=True).start()

def apply_sliders():
    j1 = s_yaw.get()
    j2 = s_shoulder.get()
    j3 = s_elbow.get()
    j4 = s_wrist.get()
    jl = s_jaw_l.get()
    jr = s_jaw_r.get()
    send_arm_command(j1, j2, j3, j4, 2)
    send_jaw_command(jl, jr, 2)
    status_var.set(f"Sent: Yaw={j1:.2f} Shoulder={j2:.2f} Elbow={j3:.2f} Wrist={j4:.2f} | Jaws={jl:.2f}/{jr:.2f}")

def home():
    for s in [s_yaw, s_shoulder, s_elbow, s_wrist, s_jaw_l, s_jaw_r]:
        s.set(0.0)
    send_arm_command(0.0, 0.0, 0.0, 0.0, 2)
    send_jaw_command(0.0, 0.0, 2)
    status_var.set("Homing robot...")

def open_jaws():
    s_jaw_l.set(0.5)
    s_jaw_r.set(-0.5)
    send_jaw_command(0.5, -0.5, 2)
    status_var.set("Opening jaws...")

def close_jaws():
    s_jaw_l.set(0.0)
    s_jaw_r.set(0.0)
    send_jaw_command(0.0, 0.0, 2)
    status_var.set("Closing jaws...")

def grasp():
    s_jaw_l.set(0.12)
    s_jaw_r.set(-0.12)
    send_jaw_command(0.12, -0.12, 2)
    status_var.set("Gentle grasp...")

# ==================== GUI ====================
root = tk.Tk()
root.title("Surgical Robot Controller")
root.geometry("700x780")
root.configure(bg="#0d1117")
root.resizable(False, False)

DARK   = "#0d1117"
CARD   = "#161b22"
BORDER = "#30363d"
ACCENT = "#58a6ff"
GREEN  = "#3fb950"
RED    = "#f85149"
YELLOW = "#d29922"
TEXT   = "#e6edf3"
MUTED  = "#8b949e"

style = ttk.Style()
style.theme_use("clam")
style.configure("TScale", background=CARD, troughcolor=BORDER, slidercolor=ACCENT)

def card(parent, title, pady=(8,8)):
    frame = tk.LabelFrame(parent, text=title,
        fg=ACCENT, bg=CARD, font=("Courier New", 10, "bold"),
        bd=1, relief="solid", padx=12, pady=8)
    frame.pack(fill="x", padx=16, pady=pady)
    return frame

def slider_row(parent, label, from_, to_, initial=0.0):
    row = tk.Frame(parent, bg=CARD)
    row.pack(fill="x", pady=3)
    tk.Label(row, text=label, fg=TEXT, bg=CARD,
             font=("Courier New", 9), width=18, anchor="w").pack(side="left")
    var = tk.DoubleVar(value=initial)
    val_lbl = tk.Label(row, textvariable=tk.StringVar(), fg=ACCENT, bg=CARD,
                       font=("Courier New", 9), width=6)

    def update_label(*_):
        val_lbl.configure(text=f"{var.get():.3f}")
    var.trace_add("write", update_label)
    update_label()

    s = tk.Scale(row, variable=var, from_=from_, to=to_,
                 orient="horizontal", resolution=0.01,
                 bg=CARD, fg=TEXT, troughcolor=BORDER,
                 activebackground=ACCENT, highlightthickness=0,
                 sliderrelief="flat", bd=0, length=340,
                 showvalue=False)
    s.pack(side="left", padx=6)
    val_lbl.pack(side="left")
    return var

# ===== HEADER =====
hdr = tk.Frame(root, bg=DARK, pady=12)
hdr.pack(fill="x")
tk.Label(hdr, text="⚕  SURGICAL ROBOT CONTROLLER",
         fg=ACCENT, bg=DARK, font=("Courier New", 14, "bold")).pack()
tk.Label(hdr, text="Laparoscopic Grasper  •  ROS2 Jazzy",
         fg=MUTED, bg=DARK, font=("Courier New", 9)).pack()

# ===== ARM JOINTS =====
arm_card = card(root, "ARM JOINTS")
s_yaw      = slider_row(arm_card, "joint1_yaw      (±90°)", -1.57, 1.57)
s_shoulder = slider_row(arm_card, "joint2_shoulder (±57°)", -1.0,  1.0)
s_elbow    = slider_row(arm_card, "joint3_elbow    (±69°)", -1.2,  1.2)
s_wrist    = slider_row(arm_card, "joint4_wrist    (360°)", -3.14, 3.14)

# ===== JAW JOINTS =====
jaw_card = card(root, "GRASPER JAWS")
s_jaw_l = slider_row(jaw_card, "jaw_left_joint",  0.0,  0.6)
s_jaw_r = slider_row(jaw_card, "jaw_right_joint", -0.6, 0.0)

# ===== SEND BUTTON =====
send_frame = tk.Frame(root, bg=DARK, pady=4)
send_frame.pack(fill="x", padx=16)
tk.Button(send_frame, text="▶  SEND TO ROBOT", command=apply_sliders,
          bg=ACCENT, fg=DARK, font=("Courier New", 11, "bold"),
          relief="flat", padx=20, pady=10, cursor="hand2",
          activebackground="#79c0ff", activeforeground=DARK).pack(fill="x")

# ===== JAW QUICK CONTROLS =====
jaw_btn_card = card(root, "JAW QUICK CONTROLS", pady=(4,4))
btn_row = tk.Frame(jaw_btn_card, bg=CARD)
btn_row.pack(fill="x")
for txt, cmd, color in [
    ("⬡ OPEN",  open_jaws,  GREEN),
    ("✦ GRASP", grasp,      YELLOW),
    ("⬟ CLOSE", close_jaws, RED),
]:
    tk.Button(btn_row, text=txt, command=cmd,
              bg=color, fg=DARK, font=("Courier New", 10, "bold"),
              relief="flat", padx=14, pady=8, cursor="hand2").pack(side="left", expand=True, fill="x", padx=4)

# ===== PRESET POSITIONS =====
preset_card = card(root, "PRESET POSITIONS", pady=(4,4))
presets = [
    ("🏠 HOME",    lambda: [s.set(v) for s,v in zip([s_yaw,s_shoulder,s_elbow,s_wrist],[0,0,0,0])] or send_arm_command(0,0,0,0)),
    ("📍 OVER PT", lambda: [s_yaw.set(0.45),s_shoulder.set(0.5),s_elbow.set(0.15),s_wrist.set(0)] or send_arm_command(0.45,0.5,0.15,0)),
    ("⬇ INSERT",  lambda: [s_yaw.set(0.45),s_shoulder.set(0.65),s_elbow.set(0.35),s_wrist.set(0)] or send_arm_command(0.45,0.65,0.35,0)),
    ("🎯 TISSUE",  lambda: [s_yaw.set(0.35),s_shoulder.set(0.68),s_elbow.set(0.40),s_wrist.set(0.3)] or send_arm_command(0.35,0.68,0.40,0.3)),
]
pr = tk.Frame(preset_card, bg=CARD)
pr.pack(fill="x")
for txt, cmd in presets:
    tk.Button(pr, text=txt, command=cmd,
              bg=BORDER, fg=TEXT, font=("Courier New", 9, "bold"),
              relief="flat", padx=8, pady=7, cursor="hand2",
              activebackground="#444c56").pack(side="left", expand=True, fill="x", padx=3)

# ===== AUTO SEQUENCE =====
seq_card = card(root, "AUTO SURGICAL SEQUENCE", pady=(4,8))
tk.Label(seq_card, text="Runs all 9 phases automatically: Home → Position → Insert → Grasp → Lift → Retract",
         fg=MUTED, bg=CARD, font=("Courier New", 8), wraplength=620).pack(pady=(0,6))
tk.Button(seq_card, text="🔬  RUN FULL SURGICAL SEQUENCE",
          command=run_full_sequence,
          bg="#238636", fg=TEXT, font=("Courier New", 11, "bold"),
          relief="flat", padx=20, pady=10, cursor="hand2",
          activebackground="#2ea043").pack(fill="x")

# ===== STATUS BAR =====
status_var = tk.StringVar(value="Ready — Use sliders or buttons to control the robot")
status_bar = tk.Frame(root, bg="#010409", pady=6)
status_bar.pack(fill="x", side="bottom")
tk.Label(status_bar, textvariable=status_var,
         fg=GREEN, bg="#010409", font=("Courier New", 9),
         anchor="w", padx=12).pack(fill="x")

root.mainloop()
