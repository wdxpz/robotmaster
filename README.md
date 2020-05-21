# 1. Hosts

* wifi network: orangecape
* robot master: 192.168.50.98
* robot nodes:
  * tb3_0: 192.168.50.54, ssh: waffle/123456
    * wifi enabler: 192.168.50.247 ssh: p1/123456
    * bt_enabler_0: 192.168.50.147 ssh:pi/123456
  * tb3_1: 192.168.50.24 ssh: robot/robot

# 2. PreSettings

## 2.1 SLAM

To get more accurate map, **cartographer** methods is recommended:

To install cartographer, see see [ROS 1 SLAM](http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#ros-1-slam) for how to install cartographer for Kinetic

to use cartographer for SLAM:

```
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
#on turtlebot node
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
#on turtlebot master
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer
$ rosrun map_server map_saver -f /save_dir/map

```

### 2.2 Navigation

the default navigation params need to modified to tune the navigation in specific environment. 

* In our case, we intend to make the distance between two adjacent robot checkpoints not too far, so we can make the `sime_time` in `turtlebot3_navigation/param/dwa_local_planner_params_$(model).yaml` smaller, in Bejing office:

```
$roscd turtlebot3_navigation
$cd param
$nano dwa_local_planner_params_waffle_pi.yaml

# Forward Simulation Parameters
  sim_time: 1.5

```

* To avoid obstacle but use limited space in Beijing office, we also modified `turtlebot3_navigation/param/costmap_common_param_$(model).yaml`

```
$roscd turtlebot3_navigation
$cd param
$nano costmap_common_params_waffle_pi.yaml

inflation_radius: 1.0
cost_scaling_factor: 10.0
```

# 3. Requirements

## master node

1. pip install -i https://pypi.tuna.tsinghua.edu.cn/simple "django<2"
2. pip install -i https://pypi.tuna.tsinghua.edu.cn/simple "djangorestframework<3.10"
3. pip install -i https://pypi.tuna.tsinghua.edu.cn/simple apscheduler
4. see [Requirement](createsite/robot_site/readme.md) for  createsite package

## turtlebot node

1. sudo apt-get install ros-<distro>-robot-upstart, replace “<distro>” by your ROS version: kinetic, melodic, …

# 4. Launch robot at turtlebot node

1. Deploy project `launch_robot` in `~/catkin_ws/src`
2. power on to auto launch the robot
2. see usage in [README.md](https://github.com/wdxpz/turtlebot_node_scripts/blob/master/README.md) of project [turtlebot_node_scripts](https://github.com/wdxpz/turtlebot_node_scripts)

# 5. Launch navigation at turtlebot master
1. deploy project `multirobot_nv` in `~/catkin_ws/src`

2. config `startall.launch` file

3. see how-to in [README.md](https://github.com/wdxpz/turtlebot_master_scripts/blob/master/README.md) of project [turtlebot_master_scripts](https://github.com/wdxpz/turtlebot_master_scripts)

4. manually launch anvigation

   ```
   roslaunch multirobot_nv startall.launch
   ```




5. launch by robotmaster service, see it in next section

# 6. robotmaster Service

## 6.1. createsite service

export current map data from map_server node, and save it in local, convert it to jpg and create the site with specific name and the jpg in server.

**required**: map_server node activated by slam or navigation

```
GET http://server_ip:port/createsite/?name=test2&desc=beijing office floor 233&forced=n

#name(required) : site name, string
#desc (optional): site description, string
#forced (optional): forced to overwrited existed site, string(['y', 'yes', 'true'] for True )
```
## 6.2. launch navigation service

```
POST http://server_ip:port/launch/

json body:
{
        'inspection_id': 103,
        'site_id': 'office12'
        'robots': {
            'robot_id1': {
                'org_pos': "(1.4, 10.5, 0)",
                'subtask': "[(1, x1, y1), (2, x2, y2), ...]"
            },
            'robot_id2': {
                'org_pos': "(5.5, 12.5, 0)",
                'subtask': "[(3, x3, y3), (4, x4, y4), ...]"
            },
            'robot_id3': {
                ...
            },
            ...
        }
    }
```

