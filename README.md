# neo_simulation2
ROS 2 simulation packages for the neobotix robots (Still in development)

## ROS 2 Installation

![ROS2_Installation Steps](https://index.ros.org/doc/ros2/Installation/Foxy/)

Note that: We support only ROS-2 Foxy (Can be installed only in Ubuntu 20.04)

## Install the relevant ROS 2 Gazebo packages:

`sudo apt-get install ros-foxy-gazebo-ros`

`sudo apt-get install ros-foxy-gazebo-plugins`

`sudo apt-get install ros-foxy-gazebo-ros-pkgs`

## Install the colcon build tool:

![Colcon_installation Steps](https://colcon.readthedocs.io/en/released/user/installation.html)

## create a ros2 workspace as follows from your terminal: 

`mkdir ros2ws`

`git clone https://github.com/neobotix/neo_simulation2.git`

## Build your workspace

`colcon build --symlink-install`

## Source your workspace

`. install/setup.bash`

## Add the model library for the Gazebo (includes all the robot meshes needed for the simulation):

`export GAZEBO_MODEL_PATH=~/ros2ws/src:$GAZEBO_MODEL_PATH `

## Launch our simulation package

`ros2 launch neo_simulation2 simulation.launch.py`

Feel free to use the teleop functionality! 

## Navigation2 

Coming Soon ! 
