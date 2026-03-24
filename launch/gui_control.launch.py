import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('laproscopic_grasper')

    # Include the main surgical robot launch
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'surgical_robot.launch.py')
        )
    )

    # Launch the GUI after a delay to ensure controllers are up
    gui_node = Node(
        package='laproscopic_grasper',
        executable='robot_controller_gui.py',
        name='robot_controller_gui',
        output='screen'
    )

    return LaunchDescription([
        main_launch,
        TimerAction(
            period=15.0,  # Wait for Gazebo and controllers to start
            actions=[gui_node]
        )
    ])
