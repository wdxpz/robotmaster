import os
import copy
import time
import pickle
import xml.etree.ElementTree as ET
from os.path import expanduser

from config import ROS_Launch_File, Map_Dir, Launch_Max_Try, Nav_Pickle_File, DEBUG
from utils.turtlebot import checkRobotNode, shell_open

#from utils.logger import logger
from utils.logger2 import getLogger
logger = getLogger('Turtlebot_Launcher')
logger.propagate = False


class Turtlebot_Launcher():
    def __init__(self, siteid, robots):
        self.robots = robots
        self.siteid = siteid

    def launch(self):
        
        launched = False
        if DEBUG:
            Launch_Max_Try = 1
            
        for i in range(Launch_Max_Try):
            logger.info('Start trial no. {} to launch navigation in multirobot mode!'.format(i+1))
            try:
                self.checkRobotsOn()
                self.startNavigation()
                time.sleep(5)
                self.checkRobotsNav()
                launched = True
                break
            except Exception as e:
                logger.info(str(e))
                msg = 'Faild of trial no. {} to launch navigation in multirobot mode'.format(i+1)
                logger.info(msg)

        if launched:
            logger.info('Succeed in trial no. {} to launch navigation in multirobot mode!'.format(i+1))
        else:
            msg = 'Faild to launch navigation in multirobot mode after {} trials'.format(Launch_Max_Try)
            logger.error(msg)
            raise Exception(msg)

    def checkRobotsOn(self):
        robot_ids = self.robots.keys()
        failed_robots = []
        
        for id in robot_ids:
            try:
                self.checkRobotOnline(id)
            except:
                failed_robots.append(id)

        if len(failed_robots) != 0:
            msg = 'robot: {} not online!'.format(failed_robots)
            logger.error(msg)
            raise Exception(msg)
    
    def checkRobotsNav(self):
        robot_ids = self.robots.keys()
        failed_robots = []
        
        for id in robot_ids:
            try:
                self.checkRobotNavOK(id)
            except:
                failed_robots.append(id)

        if len(failed_robots) != 0:
            msg = 'robot: {} navigation not ready!'.format(failed_robots)
            logger.error(msg)
            raise Exception(msg)

    def checkRobotOnline(self, robot_id):
        robot_core_node = '/{}/turtlebot3_core'.format(robot_id)
        logger.info('start to check robot {} by ping rosnode {}'.format(robot_id, robot_core_node))
        if not checkRobotNode(robot_core_node, timeout=3):
            msg = 'robot: {} not online!'.format(robot_id)
            logger.error(msg)
            raise Exception(msg)
        logger.info('robot {} is online!'.format(robot_id))
    
   

    def checkRobotNavOK(self, robot_id):
        robot_movebase_node = '/{}/move_base'.format(robot_id)
        self.logger.info('start to check robot {} by ping rosnode {}'.format(robot_id, robot_movebase_node))
        if not checkRobotNode(robot_movebase_node, timeout=3):
            msg = 'robot: {} navigation not ready, not found ()!'.format(robot_id, robot_movebase_node)
            self.logger.error(msg)
            raise Exception(msg)
        self.logger.info('robot {} navigation is ready!'.format(robot_id))
    
    def startNavigation(self):
        launch_file = self.buildLaunchFile()
        launch_file = launch_file.split('/')[-1]
        command = ['roslaunch', 'multirobot_nv',  launch_file]

        ret_code, ret_pro = shell_open(command)
        if ret_code != 0:
            msg = 'launch navigation failed, command [{}] not wokr!'.format(commnad)
            self.logger.error(msg)
            raise Exception(msg)
        else:
            with open(Nav_Pickle_File, 'wb') as f:
                pickle.dump(ret_pro, f, pickle.HIGHEST_PROTOCOL)
            



    def buildLaunchFile(self):
        org_launch_file = os.path.join(expanduser("~"), ROS_Launch_File)

        new_launch_file = org_launch_file.split('.')[0]+'_new.launch'

        tree = ET.parse(org_launch_file)
        root = tree.getroot()

        #modify mapserver node
        map_path = os.path.join(Map_Dir, self.siteid, 'map.yaml')
        mapnode = root[0]
        mapnode.getchildren()[0].attrib['value'] = map_path
        
        #create robot nodes
        robot_ids = self.robots.keys()
        for id in robot_ids:
            newnode = copy.deepcopy(root[1])
            newnode.getchildren()[0].attrib['value'] = id
            newnode.getchildren()[1].attrib['name'] = id + "_init_x"
            newnode.getchildren()[1].attrib['value'] = str(self.robots[id]['org_pos'][0])
            newnode.getchildren()[2].attrib['name'] = id + "_init_y"
            newnode.getchildren()[2].attrib['value'] = str(self.robots[id]['org_pos'][1])
            newnode.getchildren()[3].attrib['name'] = id + "_init_a"
            newnode.getchildren()[3].attrib['value'] = '0.0'
            root.append(newnode)
        #delete the template robot node
        root.remove(root[1])

        try:
            tree.write(new_launch_file)
            return new_launch_file
        except Exception as e:
            logger.error(str(e))
        