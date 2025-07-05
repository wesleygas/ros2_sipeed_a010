from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the package's share directory
    pkg_share = get_package_share_directory('sipeed_tof_ms_a010')
    
    # Path to the parameters file
    params_file = os.path.join(pkg_share, 'config', 'tof_params.yaml')

    return LaunchDescription([
        Node(
            package='sipeed_tof_ms_a010',
            executable='sipeed_tof_node',
            name='sipeed_tof_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file]
        )
    ])