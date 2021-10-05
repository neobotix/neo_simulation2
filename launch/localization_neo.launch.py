import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from pathlib import Path
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    parameters =  LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time')    
    lifecycle_nodes = ['map_server']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time' : use_sim_time,
        'yaml_filename': map_file}

    configured_params = RewrittenYaml(
        source_file=parameters,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='neo_localization2', 
            executable='neo_localization_node', 
            output='screen',
            name='neo_localization2_node', 
            parameters= [configured_params],
            remappings= remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    ])




