#!/bin/bash
machine_ip=('hostname -I')
export ROS_IP=${machine_ip[0]}

export ROS_HOSTNAME=192.168.3.90

export ROS_MASTER_URI=http://192.168.3.89:11311

export TURTLEBOT3_MODEL=waffle_pi

source /opt/ros/kinetic/setup.bash
source /home/waffle/catkin_ws/devel/setup.sh

roslaunch turtlebot3_bringup turtlebot3_robot.launch
