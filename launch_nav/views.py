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
    """
    the post data should be in json and like:
    {
        'inspection_id': 103,
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
        if 'inspection_id' not in data.keys() or 'robots' not in data.keys() :
            msg = 'key inspection_id and robots required'
            logger.info(msg)
            return Response(msg, status=status.HTTP_400_BAD_REQUEST)
        
        try: 
            inspection_id = int(data['inspection_id'])
            robots = data['robots']
            robot_ids = robots.keys()
            for id in robot_ids:  
                robots[id]
                org_pos = str(robots[id]['org_pos'])
                org_pos = ast.literal_eval(org_pos)
                robots[id]['org_pos'] = (float(org_pos[0], float(org_pos[1])))
                subtask = str(robots[id]['subtask'])
                subtask = ast.literal_eval(subtask)
                robots[id]['subtask'] = [(int(num), float(x), float(y)) for num, x, y in subtask]
        except Exception as e:
            return Response("post json data error!", status=status.HTTP_400_BAD_REQUEST)

        
        logger.info('[launch_nav] launch robot with inspection id: {}, robots: {}'.format(inspection_id, robots))
        #TODO: start multirobot navigation

        return Response({"message": "Got task data!", "data": robots}, status=status.HTTP_200_OK)
    return Response(('post robot_id and subtask to launch robot navigation'), status=status.HTTP_400_BAD_REQUEST)