# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition
import os
from pathlib import Path
import xacro

MY_NEO_ROBOT = os.environ.get('MY_ROBOT', "mpo_700")
MY_NEO_ENVIRONMENT = os.environ.get('MAP_NAME', "neo_workshop")

def execution_stage(context: LaunchContext, namespace_val):
    namespace = namespace_val.perform(context)
    namespace_topic = ""
    namespace_frame_prefix = ""

    if (namespace != ""):
        namespace_topic = "/" + namespace
        namespace_frame_prefix = namespace + "/"

    use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')
    namespace_robot = LaunchConfiguration('namespace_robot',  default="")

    x = LaunchConfiguration('x', default='0')
    y = LaunchConfiguration('y', default='0')
    z = LaunchConfiguration('z', default='0')

    urdf = os.path.join(get_package_share_directory('neo_simulation2'), 'robots/'+MY_NEO_ROBOT+'/', MY_NEO_ROBOT+'.urdf.xacro')

    spawn_entity = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        namespace=namespace_robot,
        arguments = [
            '-entity', namespace_robot,
            '-x', x,
            '-y', y,
            '-z', z, 
            '-topic', namespace_topic + "/robot_description", 
            '-robot_namespace', namespace_robot],
        output = 'screen')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=namespace_robot,
        parameters=[{'robot_description': Command([
            "xacro", " ", urdf, " ", 'robot_names:=',
            namespace_frame_prefix]),
            'frame_prefix': namespace_frame_prefix}],
        arguments=[urdf])

    teleop =  Node(
        package='teleop_twist_keyboard',executable="teleop_twist_keyboard",
        output='screen',
        prefix = 'xterm -e',
        namespace=namespace_robot,
        name='teleop')

    relay_topic = Node(
        package='topic_tools',
        executable = 'relay',
        name='relay',
        output='screen',
        parameters=[{
            'input_topic': namespace_topic + "/tf",
            'output_topic': "/tf", 
            'monitor_rate': 100.0}])

    relay_topics_tf_static = Node(
        package='topic_tools',
        executable = 'relay',
        name='relay',
        output='screen',
        parameters=[{
            'input_topic': namespace_topic + "/tf_static",
            'output_topic': "/tf_static",
            'monitor_rate': 100.0}])

    return [spawn_entity, start_robot_state_publisher_cmd, teleop, relay_topic, relay_topics_tf_static]

def generate_launch_description():
    opq_function = OpaqueFunction(function=execution_stage, args=[LaunchConfiguration('namespace_robot',  default="")])
    default_world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', MY_NEO_ENVIRONMENT + '.world')
    use_multi_robots = LaunchConfiguration('use_multi_robots', default = 'False')
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': default_world_path,
                'verbose': 'true',
            }.items(),
            condition=IfCondition(PythonExpression(['not ', use_multi_robots]))
        )

    return LaunchDescription([opq_function, gazebo])