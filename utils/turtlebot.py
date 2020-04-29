import re
import time

from subprocess import PIPE, check_output, CalledProcessError

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

def checkRobotNode(name='map_server', timeout=3):
    cmd = 'rosnode ping -c 1 {}'.format(name)

    for i in range(timeout):
        _, output = shell_cmd(cmd)
        if len(re.findall('reply', output))>0:
            return True
        time.sleep(1)

    return False