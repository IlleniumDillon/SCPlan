import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    uve_plan_dir = get_package_share_directory('uve_plan')
    config = os.path.join(uve_plan_dir, 'config', 'gen_parameter.yaml')
    
    return LaunchDescription([
        Node(
            package='uve_plan',
            executable='uve_plan_genmap',
            name='uve_plan_genmap',
            output='screen',
            parameters=[config]
        )
    ])
    