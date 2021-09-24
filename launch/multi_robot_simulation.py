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
import time

MY_NEO_ROBOT = os.environ['MY_ROBOT']
MY_NEO_ENVIRONMENT = os.environ['MAP_NAME']
MY_NO_ROBOTS = os.environ['Number_of_Robots']

def generate_launch_description():
    global MY_NO_ROBOTS 
    if(int(MY_NO_ROBOTS) > 5):
        print("Warn: Having more than 5 robots is not a good idea - too much overhead")
        print("Therefore Let's spawn 5")
        MY_NO_ROBOTS = '5'
        time.sleep(5)


    default_world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', MY_NEO_ENVIRONMENT + '.world')
    ld = LaunchDescription()
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    remapping_tf = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    remapping_cmd_vel = [('/cmd_vel', 'cmd_vel')]

    robot_dir = LaunchConfiguration(
        'robot_dir',
        default=os.path.join(get_package_share_directory('neo_simulation2'),
            'robots/'+MY_NEO_ROBOT,
            MY_NEO_ROBOT+'.urdf'))

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    rviz_config_dir = os.path.join(
        get_package_share_directory('neo_simulation2'),
        'configs',
        'default.rviz')

    urdf = os.path.join(get_package_share_directory('neo_simulation2'), 'robots/'+MY_NEO_ROBOT+'/', MY_NEO_ROBOT+'.urdf')
    
    spawn_entity = []
    
    teleop = []
    
    rviz = []

    start_robot_state_publisher_cmd = []

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': default_world_path,
                'verbose': 'true',
            }.items()
        )

    ld.add_action(gazebo)

    for i in range(0, int(MY_NO_ROBOTS)):
        spawn_entity.append(Node(package='gazebo_ros', executable='spawn_entity.py',arguments=['-entity', MY_NEO_ROBOT+ str(i) ,'-y', str(2.0 - int(i)) ,'-file', urdf, '-robot_namespace', "/"+MY_NEO_ROBOT+ str(i)], output='screen', remappings=remapping_tf))


        start_robot_state_publisher_cmd.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=MY_NEO_ROBOT+ str(i),
            parameters=[{'use_sim_time': use_sim_time,'robot_description': robot_desc}],
            arguments=[urdf],
            remappings=remapping_tf))

        teleop.append(Node(package='teleop_twist_keyboard',executable="teleop_twist_keyboard",
        output='screen',
        prefix = 'xterm -e',
        namespace=MY_NEO_ROBOT+ str(i),
        name='teleop',
        remappings=remapping_cmd_vel))

        rviz.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('neo_simulation2'), 'launch', 'rviz_launch.py')),
            launch_arguments={'namespace': MY_NEO_ROBOT+ str(i),
                              'use_namespace': "True"}.items()))

    
    for i in range(0, int(MY_NO_ROBOTS)):
        ld.add_action(spawn_entity[i])
        ld.add_action(start_robot_state_publisher_cmd[i])   
        ld.add_action(teleop[i])
        ld.add_action(rviz[i]) 

    return ld