import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("laproscopic_grasper")
    
    # pull in the main simulation launch — this kicks off Gazebo, spawns the robot, and loads controllers
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "surgical_robot.launch.py")
        ),
        launch_arguments={"enable_sofa": "false"}.items()
    )

    # venv python needed because MediaPipe isn't in the system ROS environment
    venv_python = "/home/doc/surgical_robot_ws/src/laproscopic_grasper/.venv/bin/python"

    # gesture control runs from the venv so it has access to MediaPipe
    gesture_node = ExecuteProcess(
        cmd=[venv_python, os.path.join(pkg_share, "scripts", "laproscopic_gesture_control.py")],
        output="screen",
    )



    return LaunchDescription([
        LogInfo(msg="=== STARTING COMPLETE SURGICAL SYSTEM ==="),
        
        # simulation has to be up before anything else
        simulation,

        # give Gazebo a bit of time to settle before launching the gesture node
        TimerAction(period=12.0, actions=[
            LogInfo(msg="=== STARTING TELEOPERATION & MONITORING HUB ==="),
            gesture_node,
            LogInfo(msg="=== SYSTEM FULLY OPERATIONAL ==="),
        ]),
    ])
