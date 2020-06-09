# -*- coding: utf-8 -*-
from __future__ import unicode_literals


from rest_framework.decorators import api_view
from rest_framework.decorators import parser_classes
from rest_framework import status
from rest_framework.response import Response
#from rest_framework.decorators import JSONParser

from nav_utils.turtlebot_launch import Turtlebot_Launcher
from nav_utils.turltlebot_cruise import runRoute
from nav_utils.turtlebot_robot_status import setRobotWorking, setRobotIdel, isRobotWorking

from msg_utils import putTask, getTask


from utils.logger2 import getLogger
logger = getLogger('msg_center endpoint')
logger.propagate = False


@api_view(['POST', 'GET'])
#@parser_classes([parser_classes.JSONParser])
def index(request):
    if request.method == 'POST':
        putTask(request.data)
        return Response("new task added into MSG queue!", status=status.HTTP_200_OK)
    else:
        task_data = getTask()
        return Response(data=task_data, status=status.HTTP_200_OK)