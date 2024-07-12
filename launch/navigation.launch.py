# Neobotix GmbH
# Author: Pradheep Padmanabhan

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

"""
Description:

You can launch this file using the following terminal commands:

This launch file is used to start the navigation for simulated Neobotix robot. 
It is launched after launching the "simulation.launch.py" file.

You can launch this file using the following terminal commands:

1. `ros2 launch neo_simulation2 navigation.launch.py --show-args`
   This command shows the arguments that can be passed to the launch file.
2. `ros2 launch neo_simulation2 navigation.launch.py map_name:=neo_track1 use_sim_time:=True use_multi_robots:=False use_amcl:=False`
   This command launches the simulation with sample values for the arguments.
   !(case is important for True/False)

"""

# OpaqueFunction is used to perform setup actions during launch through a Python function
def launch_setup(context: LaunchContext,
    my_neo_env_arg, use_sim_time_arg,
    use_amcl_arg, use_multi_robots_arg, namespace_arg,
    param_file_arg):

    my_neo_environment = my_neo_env_arg.perform(context)
    use_sim_time = use_sim_time_arg.perform(context)
    use_amcl = use_amcl_arg.perform(context)
    use_multi_robots = use_multi_robots_arg.perform(context)
    namespace = namespace_arg.perform(context)
    param_dir = param_file_arg.perform(context)
    
    robots = ["mpo_700", "mp_400", "mp_500", "mpo_500"]

    # Reading the selected robot from robot_name.txt
    with open('robot_name.txt', 'r') as file:
        my_neo_robot = file.read()

    if (param_dir == ""):
        for robot in robots:
            if (my_neo_robot == robot):
                param_dir = os.path.join(
                    get_package_share_directory('neo_simulation2'),
                    'configs/' + my_neo_robot,
                    'navigation.yaml')
                break

    # Only set path for the neobotix supported maps
    if (my_neo_environment == "neo_workshop" or my_neo_environment == "neo_track1"):
        map_dir = os.path.join(
                get_package_share_directory('neo_simulation2'),
                'maps',
                my_neo_environment+'.yaml')
    else:
        map_dir = my_neo_environment

    launch_actions = []

    nav2_launch_file_dir = os.path.join(get_package_share_directory('neo_nav2_bringup'), 'launch')

    # Define the IncludeLaunchDescription objects
    # neo_localization
    localization_neo_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_neo.launch.py']),
        condition=IfCondition(PythonExpression(['not ', use_amcl])),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'use_multi_robots': use_multi_robots,
            'params_file': param_dir,
            'namespace': namespace}.items(),
    )

    # amcl_localization
    localization_amcl_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_amcl.launch.py']),
        condition=IfCondition(use_amcl),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'use_multi_robots': use_multi_robots,
            'params_file': param_dir,
            'namespace': namespace}.items(),
    )

    # starting the navigation
    navigation_neo_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_neo.launch.py']),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': param_dir}.items(),
    )

    # Append the IncludeLaunchDescription objects to the launch_actions list
    launch_actions.append(localization_neo_launch_description)
    launch_actions.append(localization_amcl_launch_description)
    launch_actions.append(navigation_neo_launch_description)

    return launch_actions

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments 'my_robot' and 'map_name' with default values and descriptions
    declare_map_name_arg = DeclareLaunchArgument(
        'map', default_value='neo_workshop',
        description='Available maps: "neo_track1", "neo_workshop" or specify full path to your map'
    )

    declare_use_multi_robots_cmd = DeclareLaunchArgument(
        'use_multi_robots', default_value='False',
        description='Use multi robots "True/False"'
    )

    declare_param_file_arg = DeclareLaunchArgument(
        'param_file', 
        default_value="",
        description='Param file that needs to be used for navigation, sets according to robot by default'
    )  

    declare_use_amcl_cmd = DeclareLaunchArgument(
        'use_amcl',
        default_value='False',
        description='Use amcl for localization "True/False"'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock if true "True/False"'
    )   

    declare_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace', default_value='',
        description='Top-level namespace'
    )

    # Create launch configuration variables for the launch arguments
    my_neo_env_arg = LaunchConfiguration('map')
    use_multi_robots = LaunchConfiguration('use_multi_robots')
    use_amcl = LaunchConfiguration('use_amcl')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('robot_namespace')
    params_file = LaunchConfiguration('param_file')

    ld.add_action(declare_map_name_arg)
    ld.add_action(declare_param_file_arg)
    ld.add_action(declare_use_multi_robots_cmd)
    ld.add_action(declare_use_amcl_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)

    context_arguments = [my_neo_env_arg, use_sim_time, use_amcl, use_multi_robots, namespace, params_file]
    opq_func = OpaqueFunction(function = launch_setup, 
                            args = context_arguments)

    ld.add_action(opq_func)

    return ld
