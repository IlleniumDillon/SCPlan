import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    uve_control_dir = get_package_share_directory('uve_control')
    config = os.path.join(uve_control_dir, 'config', 'control_parameter.yaml')
    
    return LaunchDescription([
        Node(
            package='uve_control',
            executable='uve_control',
            name='uve_control',
            output='screen',
            parameters=[config]
        )
    ])
    