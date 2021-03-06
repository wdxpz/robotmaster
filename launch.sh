#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /root/catkin_ws/devel/setup.bash
#export ROS_MASTER_URI=http://192.168.27.1:11311
#export ROS_HOSTNAME=192.168.27.1
#export TURTLEBOT3_MODEL=waffle_pi
#export TZ=Asia/Shanghai

ntpdate -q ntp.ubuntu.com

#turn on bash's job control
set -m


cd /root/projects/robotmaster

#start the main process in background
python manage.py runserver 0.0.0.0:$Robotmaster_Service_Port &

#start the helper process
cd  ../taskcontroller
python main.py

