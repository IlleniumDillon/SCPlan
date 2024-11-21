import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

# uve_control
# uvs_embmaster
# uve_perception

def generate_launch_description():
    package_name = 'uve_launch'
    package_share_directory = get_package_share_directory(package_name)
    config = os.path.join(package_share_directory, 'config', 'slave_config.yaml')
    
    # uve_control
    uve_control = Node(
        package='uve_control',
        executable='uve_control',
        name='uve_control',
        output='screen',
        parameters=[config]
    )
    
    # uvs_embmaster
    uvs_embmaster = Node(
        package='uvs_embmaster',
        executable='uvs_embmaster',
        name='uvs_embmaster',
        output='screen',
        parameters=[config]
    )
    
    # uve_perception
    uve_perception = Node(
        package='uve_perception',
        executable='uve_perception',
        name='uve_perception',
        output='screen',
        parameters=[config]
    )
    
    return LaunchDescription([
        uve_control,
        uvs_embmaster,
        uve_perception
    ])