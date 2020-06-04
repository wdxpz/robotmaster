robot_status = {}

def setRobotWorking(id):
    robot_status[id] = True

def setRobotIdel(id):
    if id in robot_status.keys():
        robot_status.pop(id, None)

def isRobotWorking(id):
    if id in robot_status.keys():
        return True
    else:
        return False