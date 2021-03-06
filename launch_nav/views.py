# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import ast
import threading
import os
import pickle

from django.shortcuts import render
from rest_framework.decorators import api_view
from rest_framework.decorators import parser_classes
from rest_framework import status
from rest_framework.response import Response
#from rest_framework.decorators import JSONParser

from nav_utils.turtlebot_launch import Turtlebot_Launcher
from nav_utils.turltlebot_cruise import runRoute
from nav_utils.turtlebot_robot_status import setRobotWorking, setRobotIdel, isRobotWorking

from config import Nav_Pickle_File, Inspection_Status_Codes, Task_Type, Robot_Model
from utils.turtlebot import killNavProcess, initROSNode, checkMapFile
from utils.msg_center import addTaskIntoMsgQueue, getTasksFromMsgQueue
from utils.logger2 import getLogger
logger = getLogger('launch_av endpoint')
logger.propagate = False


@api_view(['POST'])
#@parser_classes([parser_classes.JSONParser])
def index(request):
    """
    the post data should be in json and like:
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

    """
    if request.method == 'POST':
        data = request.data
        
        if 'inspection_id' not in data.keys() or 'site_id' not in data.keys() or 'robots' not in data.keys() :
            msg = 'key inspection_id, site_id and robots required'
            logger.info(msg)
            return Response(msg, status=status.HTTP_400_BAD_REQUEST)

        for _, robot_paras in data['robots'].items():
            if set(['model', 'org_pos', 'subtask']) != set(robot_paras.keys()):
                msg = 'key model, org_pos and subtask required for each robot'
                logger.info(msg)
                return Response(msg, status=status.HTTP_400_BAD_REQUEST)
            if str(robot_paras['model']) not in Robot_Model:
                msg = 'robot model should be in: {}'.format(Robot_Model)
                logger.info(msg)
                return Response(msg, status=status.HTTP_400_BAD_REQUEST)
        
        try: 
            inspection_id = int(data['inspection_id'])
            site_id = str(data['site_id'])
            robots = data['robots']
            robot_ids = robots.keys()
            for id in robot_ids: 
                org_pos = robots[id]['org_pos'] 
                robots[id]['org_pos'] = (float(org_pos[0]), float(org_pos[1]))
                subtask = robots[id]['subtask']
                robots[id]['subtask'] = [(int(num), float(x), float(y)) for num, x, y in subtask]
        except Exception as e:
            logger.info(str(e))
            return Response("post json data error!", status=status.HTTP_400_BAD_REQUEST)

        try:
            addTaskIntoMsgQueue(data, tasktype=Task_Type['Task_Inspection'])
        except:
            return Response("Error to upload task to MSG!", status=status.HTTP_500_INTERNAL_SERVER_ERROR)
        
        return Response("Command Accepted!", status=status.HTTP_202_ACCEPTED)
        
        if not checkMapFile(site_id):
            return Response("map of {} not existed or correct map yaml failed!".format(site_id), status=status.HTTP_400_BAD_REQUEST)

        working_robots = []
        for id in robot_ids:
            if isRobotWorking(id):
                working_robots.append(id)
        if len(working_robots) > 0:
            return Response("robots {} are still working, please try again later!".format(working_robots), status=Inspection_Status_Codes['ERR_ROBOT_OCCUPIED'])

        for id in robot_ids:
            setRobotWorking(id)

        logger.info('try to kill existed navigation process before start!')
        killNavProcess()

        #init ROS node
        logger.info('Init ROS Node')
        initROSNode()
        
        logger.info('[launch_nav] launch robot with inspection id: {}, robots: {}'.format(inspection_id, robots))
        bot_launcher =Turtlebot_Launcher(site_id, robots)
        try:
            #launch navigation mode for multi-robots
            bot_launcher.launch()


            ###just for trial
            # for id in robot_ids:
            #     setRobotIdel(id)
            # robot_ids = []
            ###just for trial

            #navigate robot
            nav_tasks = []
            nav_tasks_over = {}
            for id in robot_ids:
                task_name = 'robot: {} of inpsection: {}'.format(id, inspection_id)
                nav_tasks_over[task_name] = False
            for id in robot_ids:
                #prepare cruising data
                route = []
                for pt in robots[id]['subtask']:
                    route.append(
                        {
                            'point_no': pt[0],
                            'position':{
                                'x': pt[1],
                                'y': pt[2]
                            },
                            'quaternion': {'r1': 0, 'r2': 0, 'r3': 0, 'r4': 1}
                        }
                    )
                org_pose = robots[id]['org_pos']
                task_name = 'robot: {} of inpsection: {}'.format(id, inspection_id)
                task = threading.Thread(name=task_name, target=runRoute, \
                    args=(inspection_id, id, route, org_pose, nav_tasks_over,))
                nav_tasks_over
                nav_tasks.append(task)
            for t in nav_tasks:
                logger.info("Start inspection subtask thread: {}.".format(t.getName()))
                t.setDaemon(True)
                t.start()
            msg = 'Inspection {} by robots {} started sucessfully!'.format(inspection_id, robot_ids)
            logger.info(msg)
            return Response(msg, status=Inspection_Status_Codes['INSPECTION_STARTED'])

        except Exception as e:
            logger.info('try to kill existed navigation process after failed start!')
            for id in robot_ids:
                setRobotIdel(id)
            killNavProcess()
            return Response(str(e), status=Inspection_Status_Codes['ERR_ROBOT_START'])

    return Response(('post robot_id and subtask to launch robot navigation'), status=status.HTTP_400_BAD_REQUEST)


@api_view(['GET'])
def reset(request):
    try:
        addTaskIntoMsgQueue(None, tasktype=Task_Type['Task_KillAllNavProcess'])
    except:
        return Response("Error to upload task to MSG!", status=status.HTTP_500_INTERNAL_SERVER_ERROR)
        
    return Response("Command Accepted!", status=status.HTTP_202_ACCEPTED)