# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
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

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_multi_robots = LaunchConfiguration('use_multi_robots', default='False')
    x = LaunchConfiguration('x', default='0')
    y = LaunchConfiguration('y', default='0')
    z = LaunchConfiguration('z', default='0')

    robot_name = LaunchConfiguration('namespace_robot', default="")
    context = LaunchContext()

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
        
    urdf = os.path.join(get_package_share_directory('neo_simulation2'), 'robots/'+MY_NEO_ROBOT+'/', MY_NEO_ROBOT+'.urdf.xacro')

    spawn_entity = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        namespace=robot_name,
        arguments = [
            '-entity', robot_name,
            '-x', x,
            '-y', y,
            '-z', z, 
            '-topic', "/robot_description", 
            '-robot_namespace', robot_name],
        output = 'screen')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=robot_name,
        parameters=[{'robot_description': Command([
            "xacro", " ", urdf, " ", 'robot_name:=',
            robot_name.perform(context)]),
            'frame_prefix':robot_name.perform(context)}],
        arguments=[urdf])

    teleop =  Node(
        package='teleop_twist_keyboard',executable="teleop_twist_keyboard",
        output='screen',
        prefix = 'xterm -e',
        namespace=robot_name,
        name='teleop')

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': default_world_path,
                'verbose': 'true',
            }.items(),
            condition=IfCondition(PythonExpression(['not ', use_multi_robots]))
        )

    relay_topic = Node(
        package='topic_tools',
        executable = 'relay',
        name='relay',
        output='screen',
        parameters=[{
            'input_topic': robot_name.perform(context) + "/tf",
            'output_topic': "/tf", 
            'lazy': True, 
            'stealth ': True, 
            'monitor_rate': 100.0}])

    relay_topics_tf_static = Node(
        package='topic_tools',
        executable = 'relay',
        name='relay',
        output='screen',
        parameters=[{
            'input_topic':robot_name.perform(context) + "/tf_static",
            'output_topic': "/tf_static",
            'lazy': True, 
            'stealth ': True,
            'monitor_rate': 100.0}])

    return LaunchDescription([start_robot_state_publisher_cmd, spawn_entity, teleop, gazebo, relay_topic, relay_topics_tf_static])