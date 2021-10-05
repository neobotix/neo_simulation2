# Neobotix GmbH
# Author: Pradheep Padmanabhan

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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

    ld = LaunchDescription()

    bringup = []

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('neo_simulation2'),
            'maps',
            MY_NEO_ENVIRONMENT+'.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    neo_sim_launch_file_dir = os.path.join(get_package_share_directory('neo_simulation2'), 'launch')

    for i in range(0, int(MY_NO_ROBOTS)):
        bringup.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([neo_sim_launch_file_dir, '/navigation.launch.py']),
            launch_arguments={
                'use_namespace': 'True',
                'namespace': MY_NEO_ROBOT+str(i),
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(
            get_package_share_directory('neo_simulation2'),
            'configs/'+MY_NEO_ROBOT,
            'navigation_'+ str(i) +'.yaml')}.items(),
        ))

    for i in range(0, int(MY_NO_ROBOTS)):
        ld.add_action(bringup[i])

    return ld
