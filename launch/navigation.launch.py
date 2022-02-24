# Neobotix GmbH
# Author: Pradheep Padmanabhan

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition


MY_NEO_ROBOT = os.environ['MY_ROBOT']
MY_NEO_ENVIRONMENT = os.environ['MAP_NAME']

def generate_launch_description():
    use_multi_robots = LaunchConfiguration('use_multi_robots', default='False')
    use_amcl = LaunchConfiguration('use_amcl', default='False')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='False')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('neo_simulation2'),
            'maps',
            MY_NEO_ENVIRONMENT+'.yaml'))

    param_file_name = 'navigation.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('neo_simulation2'),
            'configs/'+MY_NEO_ROBOT,
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('neo_nav2_bringup'), 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_neo.launch.py']),
            condition=IfCondition(PythonExpression(['not ', use_amcl])),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': param_dir,
                'namespace': namespace}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_amcl.launch.py']),
            condition=IfCondition(use_amcl),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': param_dir,
                'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_neo.launch.py']),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'params_file': param_dir}.items()),
        ])
