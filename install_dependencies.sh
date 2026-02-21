#!/bin/bash
# ============================================================
# LAPAROSCOPIC GRASPER SURGICAL ROBOT - DEPENDENCY INSTALLER
# ROS2 Jazzy + Gazebo Harmonic on Ubuntu 24.04 LTS
# ============================================================
# Run this script ONCE before building the workspace.
# Usage: chmod +x install_dependencies.sh && ./install_dependencies.sh
# ============================================================

set -e  # Exit on error

echo ""
echo "============================================================"
echo "  Installing Laparoscopic Grasper Robot Dependencies"
echo "  ROS2 Jazzy + Gazebo Harmonic - Ubuntu 24.04"
echo "============================================================"
echo ""

# ---- Check ROS2 Jazzy is installed ----
if ! command -v ros2 &> /dev/null; then
    echo "[ERROR] ROS2 is not installed or not sourced!"
    echo "Please install ROS2 Jazzy first:"
    echo "  https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
    echo "Then add to ~/.bashrc:"
    echo "  source /opt/ros/jazzy/setup.bash"
    exit 1
fi

ROS_DISTRO_CHECK=$(ros2 --version 2>&1 | grep -i "jazzy" || true)
echo "[INFO] ROS2 version check: $(ros2 --version 2>&1)"

# ---- Update package list ----
echo ""
echo "[1/9] Updating apt package list..."
sudo apt update

# ---- ROS2 Core packages ----
echo ""
echo "[2/9] Installing ROS2 Jazzy core packages..."
sudo apt install -y \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-rviz2

# ---- ROS2 Control ----
echo ""
echo "[3/9] Installing ros2_control stack..."
sudo apt install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-joint-state-broadcaster

# ---- Gazebo Harmonic ----
echo ""
echo "[4/9] Installing Gazebo Harmonic..."
# Add Gazebo apt repo if not already there
if ! grep -q "packages.osrfoundation.org" /etc/apt/sources.list.d/*.list 2>/dev/null; then
    sudo apt install -y wget
    sudo wget https://packages.osrfoundation.org/gazebo.gpg \
        -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt update
fi

sudo apt install -y gz-harmonic

# ---- Gazebo ROS2 Integration ----
echo ""
echo "[5/9] Installing ROS2 <-> Gazebo Harmonic bridge..."
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-gz-ros2-control

# ---- Force/Torque and sensor packages ----
echo ""
echo "[6/9] Installing sensor message packages..."
sudo apt install -y \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-trajectory-msgs \
    ros-jazzy-control-msgs \
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-diagnostic-updater

# ---- Python3 dependencies ----
echo ""
echo "[7/9] Installing Python3 dependencies..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool

# ---- Initialize rosdep ----
echo ""
echo "[8/9] Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# ---- Build tools ----
echo ""
echo "[9/9] Installing build tools..."
sudo apt install -y \
    build-essential \
    cmake \
    git

echo ""
echo "============================================================"
echo "  All dependencies installed successfully!"
echo "============================================================"
echo ""
echo "Now build the workspace:"
echo ""
echo "  cd ~/surgical_robot_ws"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo ""
echo "Then launch the simulation:"
echo ""
echo "  ros2 launch laproscopic_grasper surgical_robot.launch.py"
echo ""
echo "For RViz visualization (separate terminal):"
echo "  source ~/surgical_robot_ws/install/setup.bash"
echo "  rviz2 -d \$(ros2 pkg prefix laproscopic_grasper)/share/laproscopic_grasper/config/surgical_robot.rviz"
echo ""
