from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    cartographer_ros = Node(
            package='cartographer_ros',
            node_executable='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', get_package_share_directory('neo_simulation2') + '/configs/mpo_700',
                '-configuration_basename', 'cartographer.lua'
                ],
                # remappings=[('points2', '/scan')],
            )
    occupancy_grid_node = Node(
            package='cartographer_ros',
            node_executable='occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.02', '-publish_period_sec', '1.0']
            )
    return LaunchDescription([cartographer_ros, occupancy_grid_node])