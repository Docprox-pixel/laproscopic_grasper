"""
Trocar Gesture Control Launch

Standalone launcher for the trocar gesture controller.
Run this on its own if you just want gesture control without the full simulation.

  ros2 launch laproscopic_grasper trocar_gesture.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('laproscopic_grasper')

    # use the venv python if it exists — MediaPipe won't be on the system PATH
    workspace_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    venv_python = os.path.abspath(
        os.path.join(workspace_root, 'src', 'laproscopic_grasper', '.venv', 'bin', 'python3')
    )

    script_path = os.path.join(pkg_share, '..', '..', 'lib',
                               'laproscopic_grasper', 'trocar_gesture_control.py')

    if os.path.exists(venv_python):
        executable = venv_python
        arguments  = [script_path]
    else:
        executable = 'python3'
        arguments  = [script_path]

    return LaunchDescription([
        ExecuteProcess(
            cmd=[executable] + arguments + [
                '--ros-args',
                '-r', '__node:=trocar_gesture_controller',
                '-p', 'use_sim_time:=true',
            ],
            output='screen',
            name='trocar_gesture_control',
        )
    ])
