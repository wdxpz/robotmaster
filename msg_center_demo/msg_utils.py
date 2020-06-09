from Queue import Queue

task_msg_queue = Queue()

def putTask(task_data):
    task_msg_queue.put(task_data)

def getTask():
    if not task_msg_queue.empty():
        task_data = task_msg_queue.get_nowait()
        return task_data
    else:
        return None