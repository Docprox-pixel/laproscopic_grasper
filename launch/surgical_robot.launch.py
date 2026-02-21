"""
Launch file for Laparoscopic Grasper Surgical Robot
ROS2 Jazzy + Gazebo Harmonic

CHANGES:
  • Robot spawns ON TOP of the dark-blue patient_lower box:
      x=0.0, y=0.0, z=0.26  →  robot_base bottom at world z=1.16 m
      (world_to_base joint z=0.90 + spawn z=0.26 = 1.16 = patient_lower top)
  • ParameterValue(value_type=str) wraps Command() to avoid YAML parse error.
  • grasper_controller drives 5 joints (wrist_pitch + 4 finger joints)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory("laproscopic_grasper")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_share, "worlds", "surgical_or.world"),
        description="Path to Gazebo world file",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file   = LaunchConfiguration("world")

    xacro_file       = os.path.join(pkg_share, "urdf", "laproscopic_grasper.urdf.xacro")
    controllers_yaml = os.path.join(pkg_share, "config", "ros2_controllers.yaml")

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", xacro_file,
            " config_path:=", controllers_yaml,
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file, "--render-engine", "ogre2"],
        output="screen",
    )

    # Robot spawns at x=-1.80 over the blue table centre.
    # world_to_base in URDF: origin z=0.90 → base bottom sits on table top (z=0.90).
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",  "laproscopic_grasper",
            "-topic", "robot_description",
            "-x",     "-1.0",
            "-y",     "0.0", 
            "-z",     "0.26",
            "-R",     "0.0",
            "-P",     "0.0",
            "-Y",     "0.0",
        ],
        output="screen",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/surgical_robot/shaft_force_torque@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench",
            "/surgical_robot/jaw_left_contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            "/surgical_robot/jaw_right_contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    grasper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["grasper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Use ExecuteProcess instead of Node so that the executable path is resolved
    # at *run time*, not at LaunchDescription build time. (Node() validates the
    # executable eagerly and crashes the whole launch if the file is missing.)
    libexec_dir = PathJoinSubstitution([
        FindPackageShare("laproscopic_grasper"), "..", "..", "lib", "laproscopic_grasper"
    ])

    force_feedback_node = ExecuteProcess(
        cmd=["python3", PathJoinSubstitution([libexec_dir, "force_feedback_node.py"]),
             "--ros-args", "-r", "__node:=force_feedback_node",
             "-p", ["use_sim_time:=", use_sim_time]],
        output="screen",
    )
    grasper_motion_node = ExecuteProcess(
        cmd=["python3", PathJoinSubstitution([libexec_dir, "grasper_controller.py"]),
             "--ros-args", "-r", "__node:=grasper_motion_controller",
             "-p", ["use_sim_time:=", use_sim_time]],
        output="screen",
    )
    tissue_interaction_node = ExecuteProcess(
        cmd=["python3", PathJoinSubstitution([libexec_dir, "tissue_interaction.py"]),
             "--ros-args", "-r", "__node:=tissue_interaction_monitor",
             "-p", ["use_sim_time:=", use_sim_time]],
        output="screen",
    )


    start_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[TimerAction(period=3.0, actions=[
                joint_state_broadcaster_spawner,
                LogInfo(msg="=== Joint State Broadcaster started ==="),
            ])],
        )
    )
    start_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[TimerAction(period=2.0, actions=[
                arm_controller_spawner,
                grasper_controller_spawner,
                LogInfo(msg="=== Arm + Grasper controllers started ==="),
            ])],
        )
    )
    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        LogInfo(msg="=== Starting Laparoscopic Grasper Surgical Robot ==="),
        robot_state_publisher,
        gz_sim,
        TimerAction(period=2.0, actions=[spawn_robot]),
        ros_gz_bridge,
        start_jsb_after_spawn,
        start_arm_after_jsb,
        # Surgical Python nodes start 10 s after launch — independent of
        # controller spawn success so a controller failure doesn't silently
        # skip node startup.
        TimerAction(period=10.0, actions=[
            force_feedback_node,
            grasper_motion_node,
            tissue_interaction_node,
            LogInfo(msg="=== Surgical nodes started — System READY ==="),
        ]),
    ])
