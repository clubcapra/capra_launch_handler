#!/usr/bin/env python
from socket import timeout
from time import sleep
from xmlrpc.client import Boolean
import rospy
import rospkg
import subprocess
import xml.etree.ElementTree as ET
from capra_launch_handler.srv import LaunchRequest, LaunchListRequest
from subprocess import PIPE

launchedFiles = dict()

class LaunchMsg:
    def __init__(self):
        self.message = ""
        self.isLaunched = False
        self.fileName = ""

def launchFile(package, fileName):
    command = "roslaunch  {0} {1}".format(package, fileName)

    p = subprocess.Popen(command, shell=True)
    launchMsg = LaunchMsg()
    #Sleep to make sure the launch command has time to fail if there's an error
    sleep(1)
    state = p.poll()
    launchMsg.fileName = fileName
    if state is None:
        launchMsg.message = fileName + " was launched"
        launchedFiles[fileName] = package
        launchMsg.isLaunched = True
    else:
        launchMsg.message = fileName + " was not launched"
        launchMsg.isLaunched = False
    return launchMsg

def killLaunchFile(package, fileName):
    #Get launch file path
    rospack = rospkg.RosPack()
    path = rospack.get_path(package) + '/launch/' + fileName
    #Get all nodes from the xml launch file
    tree = ET.parse(path)
    root = tree.getroot()
    nodes = root.findall('node')
    launchMsg = LaunchMsg()
    launchMsg.fileName = fileName
    for node in nodes:
        #Kill each node
        nodeName = node.attrib['name']
        command = "rosnode kill {0}".format(nodeName)
        p = subprocess.Popen(command, shell=True)
        state = p.poll()
        if state is not None:
            launchMsg.message = "File was not killed"
            launchMsg.isLaunched = True
            return launchMsg
    launchMsg.message = "Nodes in " + fileName + " were killed"
    launchMsg.isLaunched = False
    launchedFiles.pop(fileName)
    return launchMsg

def killAll():
     for fileName, package in launchedFiles.items():
         killLaunchFile(package, fileName)

def launchCallback(data):
    package = data.package
    fileName = data.fileName
    #Check if the launch file is already running
    if fileName not in launchedFiles:
        launchMsg = launchFile(package, fileName)
    else:
        launchMsg = killLaunchFile(package, fileName)
    return launchMsg.message, launchMsg.isLaunched, launchMsg.fileName

def listCallback(data):
    return launchedFiles.values(), launchedFiles.keys()

def service():
    rospy.init_node('launchHandlerService', anonymous=True)
    rospy.on_shutdown(killAll)
    rospy.Service("launchHandler/launchFile", LaunchRequest, launchCallback)
    rospy.Service("launchHandler/getAllLaunchedFiles", LaunchListRequest, listCallback)
    rospy.spin()

if __name__ == '__main__':
    service()
