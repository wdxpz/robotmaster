# Requirements

## for robotmaster service on turtlebot master

1. pip install -i https://pypi.tuna.tsinghua.edu.cn/simple "django<2"
2. pip install -i https://pypi.tuna.tsinghua.edu.cn/simple "djangorestframework<3.10"
3. see [Requirement](createsite/robot_site/readme.md) for  createsite package

## for turtlebot

1. sudo apt-get install ros-<distro>-robot-upstart, replace “<distro>” by your ROS version: kinetic, melodic, …



# Start robot at boot on turtlebot node

## Preparation: update firewaree and software on turtlebot

refer to [section 3 How to update](https://discourse.ros.org/t/announcing-turtlebot3-software-v1-0-0-and-firmware-v1-2-0-update/4888)

## Autostart by robot_upstart 

### Tutorial

1. refer to [make a roslaunch start on boot (robot_upstart)](https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/) or [local file](docs/make roslaunch start at boot (robot_upstart).pdf)
2. [robot_upstart wiki](https://wiki.ros.org/robot_upstart)

### Shell commands

1. auto start turtlebot3 at boot on turtlebot

   ```
   $ rosrun robot_upstart install turtlebot3_bringup/launch/turtlebot3_robot.launch --job start_robot --user waffle --master http://192.168.3.89:11311 -- logdir /home/waffle/logs --symlink
   $ sudo systemctl daemon-reload
   $ sudo systemctl start start_robot
   ```


## Autostart by systemd service (works)

1.  refer to [Autostart service after system boot](https://risc.readthedocs.io/2-auto-service-start-afer-boot.html)

2. edit sh to run roslaunch robot

   ```
   #!/bin/bash
   machine_ip=('hostname -I')
   export ROS_IP=${machine_ip[0]}
   
   export ROS_HOSTNAME=192.168.3.90
   
   export ROS_MASTER_URI=http://192.168.3.89:11311
   
   export TURTLEBOT3_MODEL=waffle_pi
   
   source /opt/ros/kinetic/setup.bash
   source /home/waffle/catkin_ws/devel/setup.sh
   
   roslaunch turtlebot3_bringup turtlebot3_robot.launch
   ```

3. deploy service to start robot at boot

   ```
   [Unit]
   Description=Auto-start Robot Service
   
   [Service]
   Type=idle
   User=waffle
   Group=waffle
   ExecStart=/home/waffle/start_robot/startup_launch.sh
   Restart=on-failure
   RestartSec=10
   
   [Install]
   WantedBy=multi-user.target
   ```

4. auto start navigation on turtlebot

   **disable this, for the experimental sh and service not succeeded, and it should be launched at master node to control different robots and set their initial positions**

# Usages

## step 1: launch the robot



## step 2: run services

### 1. robotmaster service

1. createsite

   ```
   GET http://server_ip:port/createsite/?name=test2&desc=beijing office floor 233&forced=n
   
   #name(required) : site name, string
   #desc (optional): site description, string
   #forced (optional): forced to overwrited existed site, string(['y', 'yes', 'true'] for True )
   ```

   

2. : 