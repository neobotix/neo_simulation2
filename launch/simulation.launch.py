# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path

MY_NEO_ROBOT = os.environ.get('MY_ROBOT', "mpo_700")

def generate_launch_description():
    default_world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', 'neo_workshop.sdf')
    bridge_config_file = os.path.join(get_package_share_directory('neo_simulation2'), 'configs/gz_bridge', 'gz_bridge_config.yaml')
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
    urdf = os.path.join(get_package_share_directory('neo_simulation2'), 'robots/'+MY_NEO_ROBOT+'/', MY_NEO_ROBOT+'.urdf')

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        output='screen',
        arguments=['-file', urdf, '-name', "mpo_700"])

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf])

    ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ), launch_arguments={'ign_args': ['-r ', default_world_path]}.items()
        )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config_file}])
    
    teleop =  Node(package='teleop_twist_keyboard',executable="teleop_twist_keyboard",
        output='screen',
        prefix = 'xterm -e',
        name='teleop')
    
    # For harmonic and other versions GZ_SIM_RESOURCE_PATH
    # set_env_vars_resources_world = AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', 
    #         os.path.join(get_package_share_directory('neo_simulation2'), 
    #             'models'))

    # set_env_vars_resources_robots = AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', 
    #         os.path.join(get_package_share_directory('neo_simulation2')))

    return LaunchDescription([
                            # set_env_vars_resources_world,
                            # set_env_vars_resources_robots,
                            ignition,
                            spawn_robot,
                            start_robot_state_publisher_cmd,
                            gz_bridge,
                            teleop])