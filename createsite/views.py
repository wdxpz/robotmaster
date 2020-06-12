# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.shortcuts import render
from rest_framework.decorators import api_view
from rest_framework import status
from rest_framework.response import Response

from createsite import createSite, Status_Succeeded, Stauts_File_Existed, Status_Failed

#from utils.logger import logger
from utils.logger2 import getLogger

logger = getLogger('createsite endpoint')
logger.propagate = False


@api_view(['GET'])
def index(request):
    # print(request.query_params)
    if 'name' not in request.query_params.keys():
        return Response(('name required'), status=status.HTTP_400_BAD_REQUEST)
    else:
        name = request.query_params['name']
    desc = ''
    if 'desc' in request.query_params.keys():
        desc = request.query_params['desc']
    forced = True
    if 'forced' in request.query_params.keys():
        if request.query_params['forced'].lower() not in ['y', 'yes', 'true']:
            forced = False

    try:
        createSite(name, desc, forced)
        return Response("Succeed, site create!", status=status.HTTP_200_OK)
    except Exception as e:
        logger.error(str(e))
        return Response(str(e), status=status.HTTP_500_INTERNAL_SERVER_ERROR)