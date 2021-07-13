# Neobotix GmbH
# Author: Pradheep Padmanabhan

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable 
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MY_NEO_ROBOT = os.environ['MY_ROBOT'] # Set the environment variable in bashrc

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
    param_file_name = 'mapping.yaml'
    param_dir = LaunchConfiguration(
        'parameters',
        default=os.path.join(
            get_package_share_directory('neo_simulation2'),
            'configs/' + MY_NEO_ROBOT,
            param_file_name))

    slam_launch_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'parameters',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='slam_toolbox', executable='sync_slam_toolbox_node', output='screen',
            name='slam_toolbox', parameters = [param_dir])
    ])
