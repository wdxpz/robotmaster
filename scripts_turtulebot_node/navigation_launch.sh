#!/bin/bash
machine_ip=('hostname -I')
export ROS_IP=${machine_ip[0]}

export ROS_HOSTNAME=192.168.3.90

export ROS_MASTER_URI=http://192.168.3.89:11311

export TURTLEBOT3_MODEL=waffle_pi

source /opt/ros/kinetic/setup.bash
source /home/waffle/catkin_ws/devel/setup.sh

roslaunch turtlebot3_navigation turtlebot3_navigation.launch open_rviz:=false initial_pose_x:=0 initial_pose_y:=0 initial_pose_a:=0 update_min_d:=0.1 update_min_a:=0.2 map_file:=/home/waffle/map/map.yaml
