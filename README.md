# 1. Requirements

## master node

1. pip install -i https://pypi.tuna.tsinghua.edu.cn/simple "django<2"
2. pip install -i https://pypi.tuna.tsinghua.edu.cn/simple "djangorestframework<3.10"
3. see [Requirement](createsite/robot_site/readme.md) for  createsite package

## turtlebot node

1. sudo apt-get install ros-<distro>-robot-upstart, replace “<distro>” by your ROS version: kinetic, melodic, …

# Launch robot at turtlebot node

1. Deploy project `launch_robot` in `~/catkin_ws/src`
2. power on to auto launch the robot
2. see usage in [README.md](https://github.com/wdxpz/turtlebot_node_scripts/blob/master/README.md) of project [turtlebot_node_scripts](https://github.com/wdxpz/turtlebot_node_scripts)

# Launch naviagtion at turtlebot master
1. deploy project `multirobot_nv` in `~/catkin_ws/src`

2. config `startall.launch` file

3. see how-to in [README.md](https://github.com/wdxpz/turtlebot_master_scripts/blob/master/README.md) of project [turtlebot_master_scripts](https://github.com/wdxpz/turtlebot_master_scripts)

4. launch anvigation

   ```
   roslaunch multirobot_nv startall.launch
   ```

   

## robotmaster Service

1. at turtlebot node, modify `robot_id`, `ROS_HOSTNAME` and `TURTLEBOT3_MODEL` in `startup_launch.sh` if network address changes
2. ** TODO: auto launch still has some error to auto launch at boot **

##  step 2: start multiple robot navigation

```
#refer to ros project "multirobot_nav"
roslaunch multirobot_nv start.launch
```





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
