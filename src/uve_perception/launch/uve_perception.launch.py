import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    uve_perception_dir = get_package_share_directory('uve_perception')
    config = os.path.join(uve_perception_dir, 'config', 'perception_config.yaml')
    
    return LaunchDescription([
        Node(
            package='uve_perception',
            executable='uve_perception',
            name='uve_perception',
            output='screen',
            parameters=[config]
        )
    ])
    