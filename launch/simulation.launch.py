# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

"""
Description:

This launch file is used to start a ROS2 simulation for a Neobotix robot in a specified environment. 
It sets up the Gazebo simulator with the chosen robot and environment, 
optionally starts the robot state publisher, and enables keyboard teleoperation.

You can launch this file using the following terminal commands:

1. `ros2 launch neo_simulation2 simulation.launch.py --show-args`
   This command shows the arguments that can be passed to the launch file.
2. `ros2 launch neo_simulation2 simulation.launch.py my_robot:=mp_500 world:=neo_track1 use_sim_time:=true`
   This command launches the simulation with sample values for the arguments.
"""

# OpaqueFunction is used to perform setup actions during launch through a Python function
def launch_setup(context: LaunchContext, my_neo_robot_arg, my_neo_env_arg):
  # Create a list to hold all the nodes
  launch_actions = []
  # The perform method of a LaunchConfiguration is called to evaluate its value.
  my_neo_robot = my_neo_robot_arg.perform(context)
  my_neo_environment = my_neo_env_arg.perform(context)
  use_sim_time = True

  # Get the required paths for the world and robot robot_description_urdf
  if (my_neo_environment == "neo_workshop" or my_neo_environment == "neo_track1"):
    world_path = os.path.join(
      get_package_share_directory('neo_simulation2'),
      'worlds',
      my_neo_environment + '.world')
  else:
    world_path = my_neo_environment

  # Getting the robot description
  robot_description_urdf = os.path.join(
    get_package_share_directory('neo_simulation2'),
    'robots/'+my_neo_robot+'/',
    my_neo_robot+'.urdf')

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

  # Spawning the robot
  spawn_entity = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', my_neo_robot,'-file', robot_description_urdf], 
    output='screen'
  )

  # Start the robot state publisher node
  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
    arguments=[robot_description_urdf]
  )

  # Append the node to the launch_actions only if use_robot_state_pub is true
  launch_actions.append(start_robot_state_publisher_cmd)
  
  # Starting the teleop node
  teleop = Node(
    package='teleop_twist_keyboard',
    executable="teleop_twist_keyboard",
    output='screen',
    prefix = 'xterm -e',
    name='teleop'
  )

  # The required nodes can just be appended to the launch_actions list
  launch_actions.append(gazebo)
  launch_actions.append(spawn_entity)
  launch_actions.append(teleop)

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
  
  # Create launch configuration variables for the robot and map name
  my_neo_robot_arg = LaunchConfiguration('my_robot')
  my_neo_env_arg = LaunchConfiguration('world')

  ld.add_action(declare_my_robot_arg)
  ld.add_action(declare_world_name_arg)

  context_arguments = [my_neo_robot_arg, my_neo_env_arg]

  opq_function = OpaqueFunction(
      function=launch_setup, 
      args=context_arguments
  )

  ld.add_action(opq_function)

  return ld
