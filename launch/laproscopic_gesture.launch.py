import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('laproscopic_grasper')
    
    # find the venv python — pkg_share is 4 levels deep inside install/,
    # so we walk up to get back to the workspace root
    workspace_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    venv_python = os.path.join(workspace_root, 'src', 'laproscopic_grasper', '.venv', 'bin', 'python3')
    
    # normalise the path just in case there are any symlink quirks
    venv_python = os.path.abspath(venv_python)
    
    if os.path.exists(venv_python):
        executable = venv_python
        # when calling venv python directly the script goes in as an argument, not executable
        script_path = os.path.join(pkg_share, '..', '..', 'lib', 'laproscopic_grasper', 'laproscopic_gesture_control.py')
        arguments = [script_path]
    else:
        # no venv found — fall back to whatever python3 is on the PATH
        executable = 'laproscopic_gesture_control.py'
        arguments = []

    return LaunchDescription([
        Node(
            package='laproscopic_grasper',
            executable=executable,
            arguments=arguments,
            name='laproscopic_gesture_control',
            output='screen',
            parameters=[
                {'camera_index': 0},
                {'smoothing_factor': 0.15},
                {'use_sim_time': True}
            ]
        )
    ])
