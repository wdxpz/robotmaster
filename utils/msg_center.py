import requests

from config import Msg_Center_Endpoint

from utils.logger2 import getLogger
logger = getLogger('utils.msg_center')
logger.propagate = False


def addTaskIntoMsgQueue(data):

    is_error = False
    try:
        response = requests.post(Msg_Center_Endpoint, data=data)
        if response.status_code != 200:
            is_error = True
    except Exception as e:
        logger.error(str(e))
        is_error = True

    if is_error:
        msg = "Failed to upload task data to MSG center! "
        logger.error(msg)
        raise Exception("Error in upload zipped map file for robot!")

def getTasksFromMsgQueue():
    is_error = False
    try:
        response = requests.get(Msg_Center_Endpoint)
        if response.status_code != 200:
            is_error = True
    except Exception as e:
        logger.error(str(e))
        is_error = True

    if is_error:
        msg = "Failed to get new task data from MSG center! "
        logger.error(msg)
        return None
    return response.data
