# Neobotix GmbH

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import time
from launch.actions import TimerAction

import os
import time

MY_NO_ROBOTS = os.environ["Number_of_Robots"]
MY_NEO_ENVIRONMENT = os.environ.get("MAP_NAME", "neo_workshop")


def generate_launch_description():
    global MY_NO_ROBOTS

    default_world_path = os.path.join(
        get_package_share_directory("neo_simulation2"),
        "worlds",
        MY_NEO_ENVIRONMENT + ".world",
    )

    if int(MY_NO_ROBOTS) > 5:
        print("Warn: Having more than 5 robots is not a good idea - too much overhead")
        print("Therefore Let's spawn 5")
        MY_NO_ROBOTS = "5"
        time.sleep(5)

    ld = LaunchDescription()
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "world": default_world_path,
            "verbose": "true",
        }.items(),
    )
    ld.add_action(gazebo)

    spawn_actions = []

    for i in range(0, int(MY_NO_ROBOTS)):
        spawn_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("neo_simulation2"),
                        "launch",
                        "simulation.launch.py",
                    )
                ),
                launch_arguments={
                    "use_multi_robots": "True",
                    "y": str((2.0 - int(i))),
                    "namespace_robot": "robot" + str(i),
                }.items(),
            )
        )

    delay_time = 5.0
    delayed_spawn_action = [TimerAction(period=delay_time, actions=[spawn_actions[-1]])]

    for k in range(len(spawn_actions) - 2, -1, -1):
        delayed_spawn_action.append(
            TimerAction(
                period=delay_time, actions=[spawn_actions[k], delayed_spawn_action[-1]]
            )
        )

    ld.add_action(delayed_spawn_action[-1])

    return ld
