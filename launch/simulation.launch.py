# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.event_handlers import OnShutdown
import os
import xacro

"""
Description:

This launch file is used to start a ROS2 simulation for a Neobotix robot in a specified environment. 
It sets up the Gazebo simulator with the chosen robot and environment, 
optionally starts the robot state publisher, and enables keyboard teleoperation.

You can launch this file using the following terminal commands:

1. `ros2 launch neo_simulation2 simulation.launch.py --show-args`
   This command shows the arguments that can be passed to the launch file.
2. `ros2 launch neo_simulation2 simulation.launch.py my_robot:=mpo_500 world:=neo_track1 arm_type:=ur5e`
   This command launches the simulation with sample values for the arguments.
   !(only mpo_700 and mpo_500 support arms)
"""

# OpaqueFunction is used to perform setup actions during launch through a Python function
def launch_setup(context: LaunchContext, my_neo_robot_arg, my_neo_env_arg, robot_arm_arg):
    # Create a list to hold all the nodes
    launch_actions = []
    # The perform method of a LaunchConfiguration is called to evaluate its value.
    my_neo_robot = my_neo_robot_arg.perform(context)
    my_neo_environment = my_neo_env_arg.perform(context)
    robot_arm_type = robot_arm_arg.perform(context)
    use_sim_time = True

    robots = ["mpo_700", "mp_400", "mp_500", "mpo_500"]

    # Checking if the user has selected a robot that is valid
    if my_neo_robot not in robots:
        # Incase of an invalid selection
        print("Invalid option, setting mpo_700 by default")
        my_neo_robot = "mpo_700"

    with open('robot_name.txt', 'w') as file:
        file.write(my_neo_robot)

    # Remove arm_type if robot does not support it
    if (robot_arm_type != ''):
        if (my_neo_robot != "mpo_700" and my_neo_robot != "mpo_500"):
            print("Robot does not support arm, setting arm_type to empty")
            robot_arm_type = ''

    # Get the required paths for the world and robot robot_description_urdf
    if (my_neo_environment == "neo_workshop" or my_neo_environment == "neo_track1"):
        world_path = os.path.join(
            get_package_share_directory('neo_simulation2'),
            'worlds',
            my_neo_environment + '.world')
    else:
        world_path = my_neo_environment

    # Setting the world and starting the Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
        }.items()
    )

    # Getting the robot description xacro
    robot_description_xacro = os.path.join(
        get_package_share_directory('neo_simulation2'),
        'robots/'+my_neo_robot+'/',
        my_neo_robot+'.urdf.xacro')
    
    # mpo robots have the option to mount arms
    if (my_neo_robot == "mpo_700" or my_neo_robot == "mpo_500"):
        # use_gazebo is set to True since this code launches the robot in simulation
        xacro_args = {
            'use_gazebo': 'true',
            'arm_type': robot_arm_type, 
        }
    else:
        xacro_args = {
            'use_gazebo': 'true',
        }

    # Use xacro to process the file with the argunments above
    robot_description_file = xacro.process_file(
        robot_description_xacro, 
        mappings=xacro_args
        ).toxml()

    # Spawning the robot
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', my_neo_robot,'-topic', '/robot_description'], 
        output='screen'
    )

    # Start the robot state publisher node
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                    'robot_description': robot_description_file}]
    )

    # Starting the teleop node
    teleop = Node(
        package='teleop_twist_keyboard',
        executable="teleop_twist_keyboard",
        output='screen',
        prefix = 'xterm -e',
        name='teleop'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # See Issue: https://github.com/ros2/rclpy/issues/1287
    # Cannot delete the newly create file. The user has to delete it on his own
    # Refer documentation for more info
    # shutdown_event = RegisterEventHandler(
    #         OnShutdown(
    #             on_shutdown=[os.remove('robot_name.txt')]
    #         )
    #     )

    # The required nodes can just be appended to the launch_actions list
    launch_actions.append(start_robot_state_publisher_cmd)
    if robot_arm_type != '':
        launch_actions.append(joint_state_broadcaster_spawner)
        launch_actions.append(initial_joint_controller_spawner_stopped)
    launch_actions.append(gazebo)
    launch_actions.append(spawn_entity)
    launch_actions.append(teleop)

    # launch_actions.append(shutdown_event)

    return launch_actions

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments 'my_robot' and 'world' with default values and descriptions
    declare_my_robot_arg = DeclareLaunchArgument(
        'my_robot', 
        default_value='mpo_700',
        description='Robot Types: "mpo_700", "mpo_500", "mp_400", "mp_500"'
    ) 
    
    declare_world_name_arg = DeclareLaunchArgument(
        'world',
        default_value='neo_workshop',
        description='Available worlds: "neo_track1", "neo_workshop"'
    )

    declare_arm_type_cmd = DeclareLaunchArgument(
        'arm_type', default_value='',
        description='Arm Types:\n'
        '\t Elite Arms: ec66, cs66\n'
        '\t Universal Robotics: ur5, ur10, ur5e, ur10e'        
    )

    # Create launch configuration variables for the robot and map name
    my_neo_robot_arg = LaunchConfiguration('my_robot')
    my_neo_env_arg = LaunchConfiguration('world')
    robot_arm_arg = LaunchConfiguration('arm_type')

    ld.add_action(declare_my_robot_arg)
    ld.add_action(declare_world_name_arg)
    ld.add_action(declare_arm_type_cmd)

    context_arguments = [my_neo_robot_arg, my_neo_env_arg, robot_arm_arg]

    opq_function = OpaqueFunction(
        function=launch_setup, 
        args=context_arguments
    )

    ld.add_action(opq_function)

    return ld

