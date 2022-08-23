# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
import os
from pathlib import Path
import xacro

MY_NEO_ROBOT = os.environ.get('MY_ROBOT', "mpo_700")
MY_NEO_ENVIRONMENT = os.environ.get('MAP_NAME', "neo_workshop")

def generate_launch_description():
    default_world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', MY_NEO_ENVIRONMENT + '.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_names = LaunchConfiguration('robot_names', default="")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    urdf = os.path.join(get_package_share_directory('neo_simulation2'), 'robots/'+MY_NEO_ROBOT+'/', MY_NEO_ROBOT+'.urdf.xacro')
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', robot_names, '-topic', "/robot_description",  '-robot_namespace', robot_names], output='screen')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(["xacro", " ", urdf, " ", 'robot_names:=', "das/"]), 'frame_prefix':"das/"}],
        arguments=[urdf])

    teleop =  Node(package='teleop_twist_keyboard',executable="teleop_twist_keyboard",
    output='screen',
    prefix = 'xterm -e',
    namespace=robot_names,
    name='teleop')

    relay_topic = Node(
            package='topic_tools',
            executable = 'relay',
            name='relay',
            output='screen',
            parameters=[{'input_topic': "/das" + "/tf",'output_topic': "/tf", 'lazy': True, 'stealth ': True, 'monitor_rate': 100.0}])

    relay_topics_tf_static = Node(
            package='topic_tools',
            executable = 'relay',
            name='relay',
            output='screen',
            parameters=[{'input_topic': "/das" + "/tf_static",'output_topic': "/tf_static", 'lazy': True, 'stealth ': True, 'monitor_rate': 100.0}])

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': default_world_path,
                'verbose': 'true',
            }.items()
        )

    return LaunchDescription([start_robot_state_publisher_cmd, spawn_entity, teleop, gazebo, relay_topic, relay_topics_tf_static])