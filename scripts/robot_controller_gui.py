#!/usr/bin/env python3
"""
Surgical Robot GUI Controller
Directly sends joint commands via ros2 action client
Updated for KUKA iiwa14 + 5-DOF Grasper
"""

import subprocess
import threading
import tkinter as tk
from tkinter import ttk
import time

# ==================== CONTROLLER CONFIG ====================
ARM_JOINTS = [
    "iiwa14_joint1", "iiwa14_joint2", "iiwa14_joint3", 
    "iiwa14_joint4", "iiwa14_joint5", "iiwa14_joint6", "iiwa14_joint7"
]

GRASPER_JOINTS = [
    "grasper_pitch_joint",
    "grasper_yaw_joint",
    "grasper_jaw_joint"
]

def send_arm_command(positions, duration=3):
    if len(positions) != len(ARM_JOINTS):
        print(f"Error: Expected {len(ARM_JOINTS)} positions, got {len(positions)}")
        return
    
    pos_str = ", ".join([str(p) for p in positions])
    joint_names_str = ", ".join([f"'{j}'" for j in ARM_JOINTS])
    
    cmd = f"""ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{{
  trajectory: {{
    joint_names: [{joint_names_str}],
    points: [{{
      positions: [{pos_str}],
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {{sec: {duration}, nanosec: 0}}
    }}]
  }}
}}" """
    subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def send_grasper_command(positions, duration=2):
    if len(positions) != len(GRASPER_JOINTS):
        print(f"Error: Expected {len(GRASPER_JOINTS)} positions, got {len(positions)}")
        return
    
    pos_str = ", ".join([str(p) for p in positions])
    joint_names_str = ", ".join([f"'{j}'" for j in GRASPER_JOINTS])
    
    cmd = f"""ros2 action send_goal /grasper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{{
  trajectory: {{
    joint_names: [{joint_names_str}],
    points: [{{
      positions: [{pos_str}],
      velocities: [0.0, 0.0, 0.0],
      time_from_start: {{sec: {duration}, nanosec: 0}}
    }}]
  }}
}}" """
    subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def apply_sliders():
    arm_pos = [s_arm[i].get() for i in range(len(ARM_JOINTS))]
    grasper_pos = [s_grasper[i].get() for i in range(len(GRASPER_JOINTS))]
    send_arm_command(arm_pos, 2)
    send_grasper_command(grasper_pos, 2)
    status_var.set("Commands sent to controllers")

def home():
    for s in s_arm: s.set(0.0)
    for s in s_grasper: s.set(0.0)
    send_arm_command([0.0]*7, 2)
    send_grasper_command([0.0]*3, 2)
    status_var.set("Homing robot...")

def open_jaws():
    s_grasper[2].set(0.8)
    apply_sliders()
    status_var.set("Opening jaws...")

def close_jaws():
    s_grasper[2].set(0.0)
    apply_sliders()
    status_var.set("Closing jaws...")

def run_full_sequence():
    status_var.set("Running surgical sequence...")
    def sequence():
        status_var.set("Phase 1: Home")
        home()
        time.sleep(4)
        status_var.set("Phase 2: Positioning...")
        send_arm_command([0.0, 0.4, 0.0, -1.0, 0.0, 1.2, 0.0], 5)
        time.sleep(6)
        status_var.set("Phase 3: Opening...")
        send_grasper_command([0.0, 0.0, 0.8], 2)
        time.sleep(3)
        status_var.set("✓ Sequence complete!")
    threading.Thread(target=sequence, daemon=True).start()

# ==================== GUI ====================
root = tk.Tk()
root.title("⚕ Surgical Robot Controller (7-DOF Arm + 3-DOF Grasper)")
root.geometry("800x900")
root.configure(bg="#0d1117")

DARK   = "#0d1117"
CARD   = "#161b22"
BORDER = "#30363d"
ACCENT = "#58a6ff"
GREEN  = "#3fb950"
RED    = "#f85149"
YELLOW = "#d29922"
TEXT   = "#e6edf3"
MUTED  = "#8b949e"

def card(parent, title):
    frame = tk.LabelFrame(parent, text=title,
        fg=ACCENT, bg=CARD, font=("Courier New", 10, "bold"),
        bd=1, relief="solid", padx=12, pady=8)
    frame.pack(fill="x", padx=16, pady=8)
    return frame

def slider_row(parent, label, from_, to_):
    row = tk.Frame(parent, bg=CARD)
    row.pack(fill="x", pady=2)
    tk.Label(row, text=label, fg=TEXT, bg=CARD, font=("Courier New", 9), width=25, anchor="w").pack(side="left")
    var = tk.DoubleVar(value=0.0)
    val_lbl = tk.Label(row, text="0.000", fg=ACCENT, bg=CARD, font=("Courier New", 9), width=8)
    val_lbl.pack(side="right")
    def update_val(*_): val_lbl.config(text=f"{var.get():.3f}")
    var.trace_add("write", update_val)
    tk.Scale(row, variable=var, from_=from_, to=to_, orient="horizontal", resolution=0.01,
             bg=CARD, fg=TEXT, troughcolor=BORDER, activebackground=ACCENT, highlightthickness=0,
             showvalue=False, length=400).pack(side="right", padx=10)
    return var

hdr = tk.Frame(root, bg=DARK, pady=10)
hdr.pack(fill="x")
tk.Label(hdr, text="⚕ SURGICAL ROBOT CONTROLLER", fg=ACCENT, bg=DARK, font=("Courier New", 16, "bold")).pack()

arm_card = card(root, "KUKA IIWA 14 (7-DOF)")
s_arm = []
s_arm.append(slider_row(arm_card, "link1_yaw (±170°)", -2.96, 2.96))
s_arm.append(slider_row(arm_card, "link2_shoulder (±120°)", -2.09, 2.09))
s_arm.append(slider_row(arm_card, "link3_yaw (±170°)", -2.96, 2.96))
s_arm.append(slider_row(arm_card, "link4_elbow (±120°)", -2.09, 2.09))
s_arm.append(slider_row(arm_card, "link5_yaw (±170°)", -2.96, 2.96))
s_arm.append(slider_row(arm_card, "link6_wrist (±120°)", -2.09, 2.09))
s_arm.append(slider_row(arm_card, "link7_yaw (±175°)", -3.05, 3.05))

grasper_card = card(root, "LAPAROSCOPIC GRASPER (3-DOF)")
s_grasper = []
s_grasper.append(slider_row(grasper_card, "wrist_pitch (±1.57)", -1.57, 1.57))
s_grasper.append(slider_row(grasper_card, "wrist_yaw (±1.39)", -1.39, 1.39))
s_grasper.append(slider_row(grasper_card, "jaw_open (0..0.8)", 0.0, 0.8))

btn_frame = tk.Frame(root, bg=DARK, pady=10)
btn_frame.pack(fill="x", padx=16)
tk.Button(btn_frame, text="▶ APPLY SLIDERS", command=apply_sliders, bg=ACCENT, fg=DARK, font=("Courier New", 12, "bold"), pady=8).pack(fill="x", pady=4)

quick_frame = tk.Frame(root, bg=DARK)
quick_frame.pack(fill="x", padx=16)
tk.Button(quick_frame, text="🏠 HOME", command=home, bg=BORDER, fg=TEXT, width=15).pack(side="left", expand=True, padx=2)
tk.Button(quick_frame, text="⬡ OPEN", command=open_jaws, bg=GREEN, fg=DARK, width=15).pack(side="left", expand=True, padx=2)
tk.Button(quick_frame, text="⬟ CLOSE", command=close_jaws, bg=RED, fg=DARK, width=15).pack(side="left", expand=True, padx=2)

tk.Button(root, text="🔬 RUN DEMO SEQUENCE", command=run_full_sequence, bg="#238636", fg=TEXT, font=("Courier New", 11, "bold"), pady=10).pack(fill="x", padx=16, pady=20)

status_var = tk.StringVar(value="Ready")
tk.Label(root, textvariable=status_var, fg=GREEN, bg="#010409", anchor="w", padx=10, pady=5).pack(fill="x", side="bottom")

root.mainloop()
