This simulation package provides a quick and easy way to try out the autonomous mobile robots from Neobotix in ROS-2. It comes with the most commonly used configuration but is open for any kind of modification.

Please find our documentations in https://docs.neobotix.de/display/R2/ROS+2-Simulation

Dependency for this branch:

	git clone --branch $ROS_DISTRO git@github.com:ros-tooling/topic_tools.git

Currently this branch works only on Rolling distro (targetting galactic)

Launch the multi robot simulation

	ros2 launch neo_simulation2 multi_robot_simulation.py

Launch the multi robot navigation

	ros2 launch neo_simulation2 multi_robot_navigation.py

Note:: This branch is still in development! Please open an issue, if you have any feature requests! 
