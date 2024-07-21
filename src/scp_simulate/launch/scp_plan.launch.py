
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

def generate_launch_description():
    package_name = "scp_simulate"
    package_share_directory = get_package_share_directory(package_name)
    gazebo_world_file = os.path.join(package_share_directory, "worlds", "world2.world")

    ld = LaunchDescription()

    ld.add_action(
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', gazebo_world_file],
            output="screen"
        )
    )
    ld.add_action(
        Node(
            package='scp_plan',
            executable='scp_plan',
            output='screen'
        )
    )
    # ld.add_action(
    #     Node(
    #         package='scp_control',
    #         executable='scp_control',
    #         output='screen'
    #     )
    # )
    #rviz
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(package_share_directory, 'rviz', 'default.rviz')]
        )
    )

    return ld
