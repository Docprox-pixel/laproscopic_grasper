"""
Launch file for the Laparoscopic Grasper Surgical Robot
Tested on ROS2 Jazzy with Gazebo Harmonic.

Notes:
  - The robot is spawned beside the operating table, on the patient's right side.
    Took a while to get the z-height right — world_to_base joint is at z=0.90,
    so spawning at z=0.26 puts the base bottom flush with the patient_lower box top.
  - Had to wrap Command() inside ParameterValue(value_type=str) because robot_state_publisher
    was choking on the raw substitution and throwing a YAML parse error.
  - grasper_controller handles 5 joints in total: wrist_pitch + the 4 finger joints.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
    LogInfo,
)
import launch.conditions
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
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
    enable_sofa_arg = DeclareLaunchArgument(
        "enable_sofa", default_value="false",
        description="Enable high-fidelity SOFA soft tissue simulation",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file   = LaunchConfiguration("world")

    xacro_file       = os.path.join(pkg_share, "urdf",   "laproscopic_grasper.urdf.xacro")
    controllers_yaml = os.path.join(pkg_share, "config", "ros2_controllers.yaml")

    # process the xacro file at launch time so we always get fresh URDF
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", xacro_file,
        " config_path:=", controllers_yaml
    ])

    from launch_ros.parameter_descriptions import ParameterValue
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Gazebo Harmonic doesn't handle package:// URIs natively — it converts them to
    # model:// and then searches GZ_SIM_RESOURCE_PATH. So we need to point it at
    # the installed share directory so it can actually find our meshes (e.g. base_link.STL).
    gz_resource_path = os.path.join(
        os.path.dirname(pkg_share),   # …/install/laproscopic_grasper/share
    )
    # also include any Fuel/external models we've dropped into the package
    models_path = os.path.join(pkg_share, "models")
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_gz_path = gz_resource_path + ":" + models_path + (":" + existing_gz_path if existing_gz_path else "")

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file, "--render-engine", "ogre2"],
        additional_env={"GZ_SIM_RESOURCE_PATH": new_gz_path},
        output="screen",
    )

    # spawn the robot beside the operating table (patient's right side)
    # patient torso sits at roughly x=0.5, y=0, z=1.15 in world space
    # keeping z=0 so the base rests on the floor, y=0.6 gives a comfortable reach
    # yaw of -1.5708 rad (~-90°) so it faces toward the patient
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",  "laproscopic_grasper",
            "-topic", "robot_description",
            "-x",     "0.5",
            "-y",     "0.6",
            "-z",     "0.0",
            "-R",     "0.0",
            "-P",     "0.0",
            "-Y",     "-1.5708",
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
            "/world/surgical_or_world/model/laproscopic_grasper/link/jaw_left_link/sensor/jaw_left_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            "/world/surgical_or_world/model/laproscopic_grasper/link/jaw_right_link/sensor/jaw_right_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            "/surgical_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/surgical_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/grasper_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    sofa_sim_node = Node(
        package="laproscopic_grasper",
        executable="sofa_sim_node.py",
        name="sofa_simulation_node",
        output="screen",
        condition=launch.conditions.IfCondition(LaunchConfiguration("enable_sofa")),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    shaft_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["shaft_controller", "--controller-manager", "/controller_manager"],
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

    # using ExecuteProcess here instead of Node because Node() tries to validate
    # the executable at description-build time and will crash the entire launch
    # if the script isn't found yet. ExecuteProcess resolves the path lazily at runtime.
    libexec_dir = PathJoinSubstitution([
        FindPackageShare("laproscopic_grasper"), "..", "..", "lib", "laproscopic_grasper"
    ])

    force_feedback_node = ExecuteProcess(
        cmd=["python3", PathJoinSubstitution([libexec_dir, "force_feedback_node.py"]),
             "--ros-args", "-r", "__node:=force_feedback_node",
             "-r", "/surgical_robot/jaw_left_contact:=/world/surgical_or_world/model/laproscopic_grasper/link/jaw_left_link/sensor/jaw_left_contact/contact",
             "-r", "/surgical_robot/jaw_right_contact:=/world/surgical_or_world/model/laproscopic_grasper/link/jaw_right_link/sensor/jaw_right_contact/contact",
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
    force_visualizer_node = ExecuteProcess(
        cmd=["python3", PathJoinSubstitution([libexec_dir, "force_visualizer.py"]),
             "--ros-args", "-r", "__node:=force_visualizer",
             "-p", ["use_sim_time:=", use_sim_time]],
        output="screen",
    )
    grasper_command_node = ExecuteProcess(
        cmd=["python3", PathJoinSubstitution([libexec_dir, "grasper_command_node.py"]),
             "--ros-args", "-r", "__node:=grasper_command_node",
             "-p", ["use_sim_time:=", use_sim_time]],
        output="screen",
    )

    tissue_perception_node = ExecuteProcess(
        cmd=["python3", PathJoinSubstitution([libexec_dir, "tissue_perception_node.py"]),
             "--ros-args", "-r", "__node:=tissue_perception_node",
             "-p", ["use_sim_time:=", use_sim_time]],
        output="screen",
    )
    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=['0.5', '0.0', '2.0', '0', '1.5708', '0', 'world', 'surgical_camera_frame']
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
    # bring up shaft_controller first with a short delay — we want the rod
    # locked in place before gravity can pull it down. arm and grasper follow a bit later.
    start_shaft_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[TimerAction(period=0.5, actions=[
                shaft_controller_spawner,
                LogInfo(msg="=== Shaft controller started (rod locked) ==="),
            ])],
        )
    )
    start_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[TimerAction(period=2.0, actions=[
                arm_controller_spawner,
                grasper_controller_spawner,
                LogInfo(msg="=== Arm & Grasper controllers started ==="),
            ])],
        )
    )
    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        enable_sofa_arg,
        LogInfo(msg="=== Starting Laparoscopic Grasper Surgical Robot ==="),
        gz_sim,
        ros_gz_bridge,
        robot_state_publisher,
        sofa_sim_node,
        TimerAction(period=2.0, actions=[spawn_robot]),
        camera_static_tf,
        start_jsb_after_spawn,
        start_shaft_after_jsb,
        start_arm_after_jsb,
        # wait a bit to make sure the controllers are actually up before starting
        # the application nodes — a controller crash shouldn't silently skip this.
        TimerAction(period=10.0, actions=[
            force_feedback_node,
            grasper_motion_node,
            tissue_interaction_node,
            force_visualizer_node,
            grasper_command_node,
            tissue_perception_node,
            LogInfo(msg="=== Surgical Hub started — ALL NODES READY ==="),
        ]),
    ])
