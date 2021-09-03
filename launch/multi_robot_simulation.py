# Neobotix GmbH

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro

MY_NEO_ROBOT = os.environ['MY_ROBOT']
MY_NEO_ENVIRONMENT = os.environ['MAP_NAME']

def generate_launch_description():
    default_world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', MY_NEO_ENVIRONMENT + '.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_dir = LaunchConfiguration(
        'robot_dir',
        default=os.path.join(get_package_share_directory('neo_simulation2'),
            'robots/'+MY_NEO_ROBOT,
            MY_NEO_ROBOT+'.urdf'))

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    tfpre = LaunchConfiguration('tfpre')

    urdf = os.path.join(get_package_share_directory('neo_simulation2'), 'robots/'+MY_NEO_ROBOT+'/', MY_NEO_ROBOT+'.urdf')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',arguments=['-entity', MY_NEO_ROBOT+ "1", '-file', urdf, '-robot_namespace', "robot1"],output='screen')

    spawn_entity1 = Node(package='gazebo_ros', executable='spawn_entity.py',arguments=['-entity', MY_NEO_ROBOT+ "2" ,'-x', '2.0' ,'-file', urdf, '-robot_namespace', "robot2"], output='screen')

    rviz_config_dir = os.path.join(
        get_package_share_directory('neo_simulation2'),
        'configs',
        'default.rviz')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'frame_prefix': 'robot1/'}],
        arguments=[urdf])
    start_robot_state_publisher_cmd1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'frame_prefix': 'robot2/'}],
        arguments=[urdf])

    teleop =  Node(package='teleop_twist_keyboard',executable="teleop_twist_keyboard",
    output='screen',
    prefix = 'xterm -e',
    name='teleop')

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': default_world_path,
                'verbose': 'true'
            }.items()
        )

    rviz =   Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    

    return LaunchDescription([gazebo, spawn_entity,spawn_entity1, start_robot_state_publisher_cmd, start_robot_state_publisher_cmd1, teleop,rviz])