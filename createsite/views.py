# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.shortcuts import render
from django.http import HttpResponse
from rest_framework.decorators import api_view
from rest_framework import status
from rest_framework.response import Response

from robot_site.createsite import createSite, Status_Succeeded, Stauts_File_Existed, Status_Failed

from logger import logger

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
        code = createSite(name, desc, forced)
        if code == Status_Succeeded:
            return Response("Succeed, site create!", status=status.HTTP_200_OK)
        else:
            return Response("Site Existed, abort for no forced!", status=status.HTTP_202_ACCEPTED)
    except Exception as e:
        logger.error(e)
        return Response(str(e), status=status.HTTP_500_INTERNAL_SERVER_ERROR)