#!/usr/bin/env python
import sys
import os
import getopt


from map_utils import deleteRemoteSite, createRemoteSite, saveMap

import config
from utils.turtlebot import checkRobotNode
from utils.logger import logger

Status_Succeeded = 0
Stauts_File_Existed = 10
Status_Failed = 20

def createSite(sitename='test', description='', forced=True):
    if not checkRobotNode('map_server', timeout=1):
        logger.error('createsite exit! Not found map_server')
        raise Exception('not found map_server from rosnode')
        return Status_Failed

    map_path = os.path.join(config.Map_Dir, sitename)

    if os.path.exists(map_path):
        if not forced:
            logger.info('createsite exit! choose not to overwrite existed site!')
            return Stauts_File_Existed
    # logger.info(description)
    try:
        deleteRemoteSite(sitename)
        saveMap(map_path)
        createRemoteSite(sitename, description)
    except Exception as e:
        logger.error('createsite error! {}'.format(str(e)))
        raise Exception('createsite error')
        return Status_Failed

    return Status_Succeeded
    
if __name__ == '__main__':
    sitename = 'test'
    description = ''

    argv = sys.argv[1:]

    try:
        opts, args = getopt.getopt(argv, "hs:d:", ['site=', 'desc='])
    except getopt.GetoptError:
        print('createsite.py -s <sitename> -d <desciption>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('createsite.py -s <sitename> -d <desciption>')
            sys.exit()
        elif opt in ('-s', '--site'):
            sitename = arg
        elif opt in ('-d', '--desc'):
            description = arg

    createSite(sitename, description)