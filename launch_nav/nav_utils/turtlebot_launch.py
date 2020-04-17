import os
import copy
import xml.etree.ElementTree as ET
from os.path import expanduser

from config import ROS_Launch_File
from utils.turtlebot import checkRobotNode
from utils.logger import logger
logger.name= __name__

class Turtlebot_Launcher():
    def __init__(self, robots):
        self.robots = robots

    def launch(self):
        pass

    def checkRobotOnline(self, robot_id):
        robot_core_topic = '/{}/turtlebot3_core'.format(robot_id)
        if not checkRobotNode(robot_core_topic, timeout=3):
            msg = 'robot: {} not online!'.format(robot_id)
            logger.error(msg)
            raise Exception(msg)
    
    def checkRobotsOnline(self):
        robot_ids = self.robots.keys()
        failed_robots = []
        try:
            for id in robot_ids:
                self.checkRobotOnline(robot_id)
        except:
            failed_robots.append(id)

        if len(failed_robots) != 0:
            msg = 'robot: {} not online!'.format(failed_robots)
            logger.error(msg)
            raise Exception(msg)

    
    def startNavigation(self):
        pass

    def buildLaunchFile(self):
        org_launch_file = os.path.join(expanduser("~"), ROS_Launch_File)

        org_launch_file = org_launch_file.split('.')[0]+'_new.launch'

        tree = ET.parse(org_launch_file)
        root = tree.getroot()

        robot_ids = self.robots.keys()
        for id in robot_ids:
            newnode = copy.deepcopy(root[0])
            newnode.getchildren()[0].attrib['value'] = id
            newnode.getchildren()[1].attrib['name'] = id + "_init_x"
            newnode.getchildren()[1].attrib['value'] = self.robots[id]['org_pos'][0]
            newnode.getchildren()[1].attrib['name'] = id + "_init_y"
            newnode.getchildren()[2].attrib['value'] = self.robots[id]['org_pos'][1]
            newnode.getchildren()[1].attrib['name'] = id + "_init_a"
            newnode.getchildren()[3].attrib['value'] = self.robots[id]['org_pos'][2]
            root.append(newnode)
        root.remove(root[0])

        tree.write(org_launch_file)