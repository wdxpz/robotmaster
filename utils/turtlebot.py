import re
import time
import os
import rospy

from subprocess import Popen, PIPE, check_output, CalledProcessError
from config import Nav_Pickle_File

from utils.logger2 import getLogger
logger = getLogger('utils.turtlebot')
logger.propagate = False

def shell_cmd(command, shell=True):
    # result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True, shell=True)
    # if result.returncode != 0:
    #     return 1, None
    # return 0, result.stdout
    try:
        result = check_output(command, shell=shell)
        #print(result)
        return 0, result
    except CalledProcessError as e:
        return 1, str(e)

def shell_open(command):
    '''
    run shell commnad witouth waiting for its return
    '''
    try:
        process = Popen(command)
        return 0, process
    except Exception as e:
        return 1, str(e)

def checkRobotNode(name='map_server', timeout=3):
    cmd = 'rosnode ping -c 1 {}'.format(name)

    for i in range(timeout):
        _, output = shell_cmd(cmd)
        if len(re.findall('reply', output))>0:
            return True
        time.sleep(1)

    return False

def killNavProcess():
    if os.path.exists(config.Nav_Pickle_File):
        logger.info('found previous nav process, try to kill!')
        try:
            with open(config.Nav_Pickle_File, 'rb') as f:
                proc = pickle.load(f)
                proc.terminate()
        except OSError as e:
            logger.info(str(e))
        os.remove(config.Nav_Pickle_File)

def initROSNode():
    # Initialize
    #threadname = 'inspeciton_{}_robot_{}'.format(inspection_id, robot_id) 
    nodename = 'robotmaster'
    if not checkRobotNode('/'+nodename, timeout=3):
        logger.info('init node: /'+threadname)
        rospy.init_node(threadname, anonymous=False, disable_signals=True)  