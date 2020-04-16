# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import ast

from django.shortcuts import render
from rest_framework.decorators import api_view
from rest_framework.decorators import parser_classes
from rest_framework import status
from rest_framework.response import Response
#from rest_framework.decorators import JSONParser


from logger import logger

@api_view(['GET', 'POST'])
#@parser_classes([parser_classes.JSONParser])
def index(request):
    if request.method == 'POST':
        data = request.data
        if 'robot_id' not in data.keys() or 'subtask' not in data.keys():
            return Response(('robot_id: tb and subtask: [(x1, y1), ...] required'), status=status.HTTP_400_BAD_REQUEST)
        robot_id = str(data['robot_id'])
        print(robot_id)
        try:    
            subtask = str(data['subtask'])
            subtask = ast.literal_eval(subtask)
            subtask = [(float(x), float(y)) for x, y in subtask]
            print(subtask)
        except Exception as e:
            return Response("need position coordinate list for key: subtask", status=status.HTTP_400_BAD_REQUEST)

        
        logger.info('[launch_nav] launch robot with robot_id: {} ; subtask: {}'.format(robot_id, subtask))
        #TODO: start multirobot navigation

        return Response({"message": "Got some data!", "data": subtask})
    return Response(('post robot_id and subtask to launch robot navigation'), status=status.HTTP_400_BAD_REQUEST)
