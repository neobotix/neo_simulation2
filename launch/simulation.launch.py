# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, Command
from launch_ros.actions import Node
import os
from pathlib import Path
import xacro
from launch.launch_context import LaunchContext

MY_NEO_ROBOT = os.environ.get('MY_ROBOT', "mpo_700")
MY_NEO_ENVIRONMENT = os.environ.get('MAP_NAME', "neo_workshop")

def execution_stage(context: LaunchContext):
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
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

    urdf = os.path.join(
        get_package_share_directory('neo_simulation2'),
        'robots/'+MY_NEO_ROBOT+'/', MY_NEO_ROBOT+'.urdf.xacro')
    
    # ToDo add an launch argument for include_arm
    xacro_file = Command([
            "xacro", " ", urdf, " ", 'include_arm:=',
            "true"])

    doc = xacro.parse(xacro_file.perform(context)) 
    xacro.process_doc(doc) 

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', MY_NEO_ROBOT, '-topic', "robot_description"],
        output='screen')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': doc.toxml()}],
    )

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
                'verbose': 'true',
            }.items()
        )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    return [spawn_entity, start_robot_state_publisher_cmd, teleop, gazebo, initial_joint_controller_spawner_stopped, joint_state_broadcaster_spawner]


def generate_launch_description():
    opq_function = OpaqueFunction(function=execution_stage)
    return LaunchDescription([opq_function])
    