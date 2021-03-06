
import os
import yaml
from Queue import Queue
from robotmaster.settings import BASE_DIR

DEBUG = False

PI = 3.1415926535897

#logger
log_file = os.path.join(os.path.dirname(BASE_DIR), 'robot_master.log')

#map service
Map_Dir = os.path.join(os.path.dirname(BASE_DIR), 'map')
Create_Site_Endoint = 'http://www.bestfly.ml:8000/site/'
Delete_Site_Endpoint = 'http://www.bestfly.ml:8000/site/'

#robot launch configuration
ROS_Launch_File = 'catkin_ws/src/multirobot_nv/launch/startall.launch'
Launch_Max_Try = 3

#robot pose initialization configration
Trial_Set_Pose_Count = 3

#robot navigation configuration
Wait_For_GoToPose_Time = 60
Holding_Step_Time = 20
Holding_Time_Variance = 1
Circle_Rotate_Steps = 4
Rotate_Speed = 30
Valid_Range_Radius = 0.1
Holding_Time = Holding_Step_Time+360/Rotate_Speed+Holding_Time_Variance
##time interval to upload to tsdb
Pos_Collect_Interval = 0.2
Upload_Interval = 2
##navigation prcocess pickle file
Nav_Pickle_File = os.path.join(BASE_DIR, 'nav_process.pkl')
#tsdb
upload_URL = 'www.bestfly.ml'
upload_PORT = 8086
upload_DB = 'robot'
Table_Name_Robot_Pos = 'robot_poss'
Table_Name_Robot_Event = 'robot_event'

#Inspection Status Update Entrypoint
Inspection_Status_Endpoint= 'http://www.bestfly.ml:8000/inspection/'

#load constant varibles
constants_yaml = os.path.join(os.path.dirname(BASE_DIR), 'constants.yml')
with open(constants_yaml, "rb") as f:
    constants_data = yaml.load(f)
    Task_Type = constants_data['Task_Type']
    Inspection_Status_Codes = constants_data['Inspection_Status_Codes']
    Msg_Center_Url = constants_data['Msg_Center']
    Robot_Model = constants_data['Robot_Model']

#MSG center entrypoint
Robotmaster_Service_Port=8100
if 'Robotmaster_Service_Port' in os.environ.keys():
    Robotmaster_Service_Port = os.environ['Robotmaster_Service_Port']
Msg_Center_Endpoint=Msg_Center_Url + str(Robotmaster_Service_Port) + '/tasks/'
task_msg_queue = Queue()