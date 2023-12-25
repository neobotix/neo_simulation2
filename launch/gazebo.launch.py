# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition
import os
from pathlib import Path
import xacro

MY_NEO_ROBOT = os.environ.get('MY_ROBOT', "mpo_700")
MY_NEO_ENVIRONMENT = os.environ.get('MAP_NAME', "neo_workshop")

def generate_launch_description():
    default_world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', MY_NEO_ENVIRONMENT + '.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': default_world_path,
            'verbose': 'true',
        }.items()
    )

    return LaunchDescription([gazebo])