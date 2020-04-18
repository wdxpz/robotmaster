
import os
from robotmaster.settings import BASE_DIR

PI = 3.1415926535897

#logger
log_file = os.path.join(BASE_DIR, 'robot_master.log')

#map service
Map_Dir = os.path.join(BASE_DIR, 'map')
Create_Site_Endoint = 'http://www.bestfly.ml:8000/site/'
Delete_Site_Endpoint = 'http://www.bestfly.ml:8000/site/'

#robot launch configuration
ROS_Launch_File = 'catkin_ws/src/multirobot_nv/launch/startall.launch'
Launch_Max_Try = 5

#robot navigation configuration

Holding_Step_Time = 20
Holding_Time_Variance = 1
Circle_Rotate_Steps = 4
Rotate_Speed = 30
Valid_Range_Radius = 0.1
Holding_Time = Holding_Step_Time+360/Rotate_Speed+Holding_Time_Variance
##time interval to upload to tsdb
Pos_Collect_Interval = 0.2
Upload_Interval = 2

#tsdb
upload_URL = 'www.bestfly.ml'
upload_PORT = 8086
upload_DB = 'robot'
Table_Name_Robot_Pos = 'robot_poss'
Table_Name_Robot_Event = 'robot_event'
