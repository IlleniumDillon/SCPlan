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

# uvs_optitrack
# uvs_mapserver
# uve_mapclient
# uve_plan
# rviz2
def generate_launch_description():
    package_name = 'uve_plan'
    package_share_directory = get_package_share_directory(package_name)
    config = os.path.join(package_share_directory, 'config', 'layer3_test.yaml')
    
    rviz_config_share = get_package_share_directory("uve_launch")
    
    # uvs_optitrack
    uvs_optitrack = Node(
        package='uve_mapclient',
        executable='uve_simoptitrack',
        name='uve_simoptitrack',
        output='screen',
        parameters=[config]
    )
    
    # uvs_mapserver
    uvs_mapserver = Node(
        package='uvs_mapserver',
        executable='uvs_mapserver',
        name='uvs_mapserver',
        output='screen',
        parameters=[config]
    )
    
    # uve_mapclient
    uve_mapclient = Node(
        package='uve_mapclient',
        executable='uve_mapclient',
        name='uve_mapclient',
        output='screen',
        parameters=[config]
    )
    
    uve_plan = Node(
        package='uve_plan',
        executable='uve_layer3_plan',
        name='uve_layer3_plan',
        output='screen',
        parameters=[config]
        
    )
    # uve_plan = ExecuteProcess(
    #     cmd=['ros2', 'run', '--prefix', "'gdbserver localhost:6688'", 'uve_plan', 'uve_layer2_plan', 
    #          '--ros-args', '-r', '__node:=uve_layer2_plan','--params-file', config],
    #     output='screen'
    # )
    
    # rviz2
    rviz2 = ExecuteProcess(
        cmd=['rviz2', '-d', os.path.join(rviz_config_share, 'rviz', 'host.rviz')],
        output='screen'
    )
    
    ld = LaunchDescription(
        [
            uvs_optitrack,
            uvs_mapserver,
            uve_mapclient,
            # uve_plan,
            rviz2
        ]
    )
    
    return ld