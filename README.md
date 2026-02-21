# 🔬 Laparoscopic Grasper Surgical Robot
## ROS2 Jazzy + Gazebo Harmonic Simulation

A complete surgical robot simulation featuring a **fixed-base laparoscopic grasper**  
that enters a simulated body cavity, grasps tissue, and provides **real-time force feedback**.

---

## 🏗️ Robot Architecture

```
WORLD (fixed)
  │
  └─ robot_base (heavy floor-bolted base)
       │
       └─ base_column
            │
            └─ [joint1_yaw] → arm_link1      ← Rotation sweep (±90°)
                 │
                 └─ [joint2_shoulder] → arm_link2   ← Up/down pitch (±57°)
                      │
                      └─ [joint3_elbow] → arm_link3  ← Elbow flex (±69°)
                           │
                           └─ [joint4_wrist] → trocar_shaft  ← Wrist roll (full 360°)
                                │
                                ├─ force_sensor_ring  ← 🟡 F/T sensor at shaft
                                │
                                └─ jaw_base
                                     ├─ [jaw_left_joint]  → jaw_left
                                     │    └─ jaw_left_sensor   ← 🟡 Contact sensor
                                     └─ [jaw_right_joint] → jaw_right
                                          └─ jaw_right_sensor  ← 🟡 Contact sensor
```

---

## 🏥 OR Environment

The Gazebo world contains a complete simulated operating room:

| Object | Description |
|--------|-------------|
| **OR Floor** | Anti-slip surgical floor |
| **Walls** | 4 walls forming the OR |
| **Surgical Table** | Stainless steel table with green mattress |
| **Patient Body** | Torso, head, lower body with surgical drape |
| **Tissue Target** | Red ellipsoid tissue that the grasper picks up |
| **IV Stand** | Standard IV pole |
| **Anesthesia Machine** | Equipment beside table |
| **Monitor Screen** | Surgical display |
| **Overhead Surgical Light** | Spot light directly over table |

---

## 🔧 Force Feedback Sensors

### 1. Shaft Force/Torque Sensor (`force_sensor_ring`)
- **Type**: Force/Torque (6-axis)
- **Location**: Along trocar shaft at body entry point
- **Rate**: 100 Hz
- **Topic**: `/surgical_robot/shaft_force_torque`

### 2. Left Jaw Contact Sensor (`jaw_left_sensor`)
- **Type**: Contact/Pressure
- **Location**: Tip of left grasper jaw
- **Rate**: 100 Hz
- **Topic**: `/surgical_robot/jaw_left_contact`

### 3. Right Jaw Contact Sensor (`jaw_right_sensor`)
- **Type**: Contact/Pressure
- **Location**: Tip of right grasper jaw
- **Rate**: 100 Hz
- **Topic**: `/surgical_robot/jaw_right_contact`

---

## 📡 ROS2 Topics

### Subscribed by nodes:
| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | All joint positions, velocities, efforts |
| `/surgical_robot/shaft_force_torque` | `geometry_msgs/WrenchStamped` | Raw F/T data |
| `/surgical_robot/jaw_left_contact` | Contact | Left jaw contact events |
| `/surgical_robot/jaw_right_contact` | Contact | Right jaw contact events |

### Published by nodes:
| Topic | Type | Description |
|-------|------|-------------|
| `/surgical_robot/grip_force` | `std_msgs/Float32` | Estimated grip force in Newtons |
| `/surgical_robot/safety_alert` | `std_msgs/String` | OK / WARNING / CRITICAL |
| `/surgical_robot/tissue_status` | `std_msgs/String` | Current tissue handling status |
| `/surgical_robot/force_feedback` | `geometry_msgs/WrenchStamped` | Processed feedback |
| `/surgical_robot/interaction_report` | `std_msgs/String` | Contact quality report |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Full system diagnostics |

### Action servers (controllers):
| Action | Description |
|--------|-------------|
| `/arm_controller/follow_joint_trajectory` | Move arm joints |
| `/grasper_controller/follow_joint_trajectory` | Open/close jaws |

---

## 🚀 Installation & Running

### Step 1 — Install dependencies (run ONCE)
```bash
chmod +x install_dependencies.sh
./install_dependencies.sh
```

### Step 2 — Build workspace
```bash
cd ~/surgical_robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Step 3 — Add to .bashrc (optional but recommended)
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/surgical_robot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 4 — Launch simulation
```bash
ros2 launch laproscopic_grasper surgical_robot.launch.py
```

### Step 5 — Open RViz (new terminal)
```bash
source ~/surgical_robot_ws/install/setup.bash
rviz2 -d $(ros2 pkg prefix laproscopic_grasper)/share/laproscopic_grasper/config/surgical_robot.rviz
```

---

## 📊 Monitoring Commands

```bash
# Watch grip force in real-time
ros2 topic echo /surgical_robot/grip_force

# Watch safety alerts
ros2 topic echo /surgical_robot/safety_alert

# Watch raw F/T sensor
ros2 topic echo /surgical_robot/shaft_force_torque

# Watch tissue interaction quality
ros2 topic echo /surgical_robot/interaction_report

# View all joint states
ros2 topic echo /joint_states

# System diagnostics
ros2 topic echo /diagnostics

# List all active topics
ros2 topic list

# Check controllers
ros2 control list_controllers

# Check hardware interfaces
ros2 control list_hardware_interfaces
```

---

## 🎬 Surgical Motion Sequence

The robot automatically executes this sequence after startup:

| Phase | Action | Duration |
|-------|--------|----------|
| 1 | Home position | 2s |
| 2 | Sweep arm over patient | 6s |
| 3 | Open grasper jaws | 2s |
| 4 | Insert trocar into body cavity | 5s |
| 5 | Navigate to tissue target | 5s |
| 6 | Close jaws — GRASP tissue | 3s |
| 7 | Lift tissue upward | 7s |
| 8 | Release tissue | 2s |
| 9 | Retract from body to home | 9s |

---

## ⚠️ Safety Thresholds

| Level | Force | Action |
|-------|-------|--------|
| ✅ SAFE | < 1.5 N | Normal operation |
| ⚠️ WARNING | 1.5 – 2.0 N | Slow down, reduce grip |
| 🔴 CRITICAL | > 2.0 N | Emergency stop, release tissue |

Thresholds are tissue-type specific. Modify in `tissue_interaction.py`.

---

## 🛠️ Troubleshooting

**Gazebo doesn't start:**
```bash
# Check Gazebo Harmonic installation
gz sim --version
# Should show: Gazebo Harmonic
```

**Controllers not loading:**
```bash
# Check controller manager
ros2 control list_controllers
# If empty, check ros2_controllers.yaml path in launch file
```

**Robot spawns but doesn't move:**
```bash
# Check if action servers are up
ros2 action list
# Should show arm_controller and grasper_controller actions
```

**Build errors with xacro:**
```bash
sudo apt install ros-jazzy-xacro
```

**gz_ros2_control not found:**
```bash
sudo apt install ros-jazzy-gz-ros2-control
```

---

## 📦 Package Structure

```
laproscopic_grasper/
├── urdf/
│   └── laproscopic_grasper.urdf.xacro    ← Robot description
├── launch/
│   └── surgical_robot.launch.py          ← Main launch file
├── config/
│   ├── ros2_controllers.yaml             ← Controller config
│   └── surgical_robot.rviz              ← RViz config
├── worlds/
│   └── surgical_or.world                 ← Gazebo OR environment
├── scripts/
│   ├── force_feedback_node.py            ← Sensor processing
│   ├── grasper_controller.py             ← Motion sequences
│   └── tissue_interaction.py            ← Interaction monitoring
├── CMakeLists.txt
└── package.xml
```
