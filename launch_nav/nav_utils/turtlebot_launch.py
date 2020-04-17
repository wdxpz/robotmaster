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

        ff = '/home/sw/startall_a.launch'

        tree = ET.parse(f)

        root = tree.getroot()

        newnode = copy.deepcopy(root[0])

        newnode.getchildren()[0].attrib['value'] = 'tb3_1'
        newnode.getchildren()[1].attrib['value'] = '1.0'
        newnode.getchildren()[2].attrib['value'] = '2.0'
        newnode.getchildren()[3].attrib['value'] = '3.0'

        root.append(newnode)

        print(root[1].attrib['file'])

        print(root[1].getchildren()[0].attrib['name'])

        print(root[1].getchildren()[0].attrib['value'])

        tree.write(ff)