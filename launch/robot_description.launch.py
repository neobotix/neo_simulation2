import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.actions import TimerAction
import os
from pathlib import Path
import xacro
"""
This code is used for debugging, quick testing, and visualization of the robot with arm and gripper in Rviz. 
"""

# OpaqueFunction is used to perform setup actions during launch through a Python function
def launch_setup(context, use_sim_time_arg, use_joint_state_publisher_gui_arg, my_neo_robot_arg, robot_arm_arg):
    # Create a list to hold all the nodes
    launch_actions = []

    robot_description_pkg = get_package_share_directory('neo_simulation2')

    use_sim_time = use_sim_time_arg.perform(context).lower() == 'true'
    my_neo_robot = my_neo_robot_arg.perform(context)
    arm_type = robot_arm_arg.perform(context)

    # IfCondition only accepts "True" or "False" 
    use_joint_state_publisher_gui = str(use_joint_state_publisher_gui_arg.perform(context).lower() == 'true')

    # Getting the robot description xacro
    robot_description_xacro = os.path.join(
        get_package_share_directory('neo_simulation2'),
        'robots/'+my_neo_robot+'/',
        my_neo_robot+'.urdf.xacro')
    
     # use_gazebo is set to True since this code launches the robot in simulation
    xacro_args = {
        'use_gazebo': 'false',
        'arm_type': arm_type,
        'use_docking_adapter': 'false'
    }

    # Use xacro to process the file with the argunments above
    robot_description_file = xacro.process_file(
        robot_description_xacro, 
        mappings=xacro_args
        ).toxml()

    rviz_config = os.path.join(robot_description_pkg, 'rviz', 'robot_description_rviz.rviz')

    # Start the joint state publisher gui only if use_joint_state_publisher_gui is True
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_joint_state_publisher_gui),
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=IfCondition(PythonExpression(['not ', use_joint_state_publisher_gui])),
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robot_description_file, 'use_sim_time': use_sim_time}],
    )

    # Rviz node
    start_rviz_cmd = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + rviz_config]
    )

    launch_actions.append(start_rviz_cmd)
    launch_actions.append(start_robot_state_publisher_cmd)
    launch_actions.append(start_joint_state_publisher_cmd)
    launch_actions.append(start_joint_state_publisher_gui_cmd)

    return launch_actions

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments with default values and descriptions
    declare_use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Use simulation clock if True (True/False)'
        )
    declare_use_joint_state_publisher_gui_arg = DeclareLaunchArgument(
            'use_joint_state_publisher_gui', default_value='True',
            description='Use joint state publisher gui if True (True/False)'
        )
    declare_my_robot_arg = DeclareLaunchArgument(
        'my_robot', 
        default_value='mpo_700',
        description='Robot Types: "mpo_700", "mpo_500", "mp_400", "mp_500"'
        )
    declare_arm_type_cmd = DeclareLaunchArgument(
        'arm_type', default_value='',
        description='Arm Types:\n'
        '\t Elite Arms: ec66, cs66\n'
        '\t Universal Robotics: ur5, ur10, ur5e, ur10e'        
        )

    use_sim_time_arg = LaunchConfiguration('use_sim_time')
    use_joint_state_publisher_gui_arg = LaunchConfiguration('use_joint_state_publisher_gui')
    robot_arm_arg = LaunchConfiguration('arm_type')
    my_neo_robot_arg = LaunchConfiguration('my_robot')

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_use_joint_state_publisher_gui_arg)
    ld.add_action(declare_my_robot_arg)
    ld.add_action(declare_arm_type_cmd)

    context_arguments = [use_sim_time_arg, use_joint_state_publisher_gui_arg, my_neo_robot_arg, robot_arm_arg]
    opq_func = OpaqueFunction(  
        function = launch_setup,
        args = context_arguments
        )

    ld.add_action(opq_func)

    return ld
