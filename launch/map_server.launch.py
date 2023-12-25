# Neobotix GmbH
# Author: Pradheep Padmanabhan

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

MY_NEO_ENVIRONMENT = os.environ.get('MAP_NAME', "neo_track1")

def generate_launch_description():
    ld = LaunchDescription()
    bringup = []
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('neo_simulation2'),
            'maps',
            MY_NEO_ENVIRONMENT+'.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    neo_sim_launch_file_dir = os.path.join(get_package_share_directory('neo_simulation2'), 'launch')
    
    ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([neo_sim_launch_file_dir, '/neo_map_server.launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                }.items(),
        ))

    return ld