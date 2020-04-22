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

4. manually launch anvigation

   ```
   roslaunch multirobot_nv startall.launch
   ```




5. launch by robotmaster service, see it in next section

# robotmaster Service

## 1. createsit service

export current map data from map_server node, and save it in local, convert it to jpg and create the site with specific name and the jpg in server.

**required**: map_server node activated by slam or navigation

```
GET http://server_ip:port/createsite/?name=test2&desc=beijing office floor 233&forced=n

#name(required) : site name, string
#desc (optional): site description, string
#forced (optional): forced to overwrited existed site, string(['y', 'yes', 'true'] for True )
```
## 2. launch navigation service

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

