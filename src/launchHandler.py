#!/usr/bin/env python
from time import sleep
import rospy
import signal
import subprocess
import os
from capra_launch_handler.srv import LaunchRequest, LaunchListRequest

launchedFiles = dict()

class LaunchFile: 
    def __init__(self, package, fileName, pid):
        self.package = package
        self.fileName = fileName
        self.pid = pid
class LaunchMsg:
    def __init__(self):
        self.message = ""
        self.isLaunched = False
        self.fileName = ""

def launchFile(package, fileName):
    command = "roslaunch  {0} {1}".format(package, fileName)

    p = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

    launchMsg = LaunchMsg()
    #Sleep to make sure the launch command has time to fail if there's an error
    sleep(1)
    state = p.poll()
    launchMsg.fileName = fileName
    if state is None:
        launchMsg.message = fileName + " was launched"
        launchedFiles[fileName] = LaunchFile(package, fileName, p.pid)
        launchMsg.isLaunched = True
    else:
        launchMsg.message = fileName + " was not launched"
        launchMsg.isLaunched = False
    return launchMsg

def killLaunchFile(fileName):
    launchMsg = LaunchMsg()
    launchMsg.fileName = fileName
    if fileName in launchedFiles:
        pid = launchedFiles[fileName].pid
        os.killpg(os.getpgid(pid), signal.SIGINT)
        del launchedFiles[fileName]
        launchMsg.message = fileName + " was killed"
        launchMsg.isLaunched = False
    else:
        launchMsg.message = fileName + " was not launched"
        launchMsg.isLaunched = False
    return launchMsg

def killAll():
     for launchFile in launchedFiles.values():
         killLaunchFile(launchFile.fileName)

def launchCallback(data):
    package = data.package
    fileName = data.fileName
    #Check if the launch file is already running
    if fileName not in launchedFiles:
        launchMsg = launchFile(package, fileName)
    else:
        launchMsg = killLaunchFile(fileName)
    return launchMsg.message, launchMsg.isLaunched, launchMsg.fileName

def getLaunchedFiles(data):
    #Get array of package names
    packageNames = []
    for launchedFile in launchedFiles.values(): 
        packageNames.append(launchedFile.package)

    return packageNames, launchedFiles.keys()

def service():
    rospy.init_node('launchHandlerService', anonymous=True)
    rospy.on_shutdown(killAll)
    rospy.Service("launchHandler/launchFile", LaunchRequest, launchCallback)
    rospy.Service("launchHandler/getAllLaunchedFiles", LaunchListRequest, getLaunchedFiles)
    rospy.spin()

if __name__ == '__main__':
    service()
