import config

def putTask(task_data):
    config.task_msg_queue.put(task_data)

def getTask():
    if not config.task_msg_queue.empty():
        task_data = config.task_msg_queue.get_nowait()
        return task_data
    else:
        return None