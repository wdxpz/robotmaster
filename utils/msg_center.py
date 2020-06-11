import requests
import json

from config import Msg_Center_Endpoint

from utils.logger2 import getLogger
logger = getLogger('utils.msg_center')
logger.propagate = False


def addTaskIntoMsgQueue(data, tasktype=-1):

    is_error = False
    try:
        response = requests.post(Msg_Center_Endpoint, data={tasktype:json.dumps(data)})
        if response.status_code != 200:
            is_error = True
    except Exception as e:
        logger.error(str(e))
        is_error = True

    if is_error:
        msg = "Failed to upload task data to MSG center! "
        logger.error(msg)
        raise Exception("Error in upload zipped map file for robot!")
    
    logger.info("Succeeded to upload task data to MSG center!")

def getTasksFromMsgQueue():
    is_error = False
    try:
        response = requests.get(Msg_Center_Endpoint)
        if response.status_code != 200:
            is_error = True
        data = json.loads(response.content)['data']
        task_data = json.loads(data)
    except Exception as e:
        logger.error(str(e))
        is_error = True

    if is_error:
        msg = "Failed to get new task data from MSG center! "
        logger.error(msg)
        return None
    

    logger.info("Succeeded to get new task data from MSG center!")
    return task_data

