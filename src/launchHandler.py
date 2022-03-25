#!/usr/bin/env python
from time import sleep
from xmlrpc.client import Boolean
import rospy
import rospkg
import subprocess
import xml.etree.ElementTree as ET
from capra_launch_handler.srv import LaunchRequest, LaunchListRequest

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
    isLaunched = False
    isLaunched = recursivekillLaunchFile(package, fileName)
    launchMsg = LaunchMsg()
    launchMsg.fileName = fileName
    if(not isLaunched):
        launchedFiles.pop(fileName)
        launchMsg.message = "Nodes in " + fileName + " were killed"
        launchMsg.isLaunched = False
    else:
        launchMsg.message = "Nodes in " + fileName + " were not killed"
        launchMsg.isLaunched = True
    return launchMsg
    
def recursivekillLaunchFile(package, fileName):
    #Get launch file path
    rospack = rospkg.RosPack()
    path = rospack.get_path(package) + '/launch/' + fileName
    #Get all nodes from the xml launch file
    tree = ET.parse(path)
    root = tree.getroot()
    nodes = root.findall('node')
    includes = root.findall('include')
    #Find robot_namespace arg
    robot_namespace = root.find('arg[@name="robot_namespace"]')
    isLaunched = False
    for include in includes:
        #Find the file to include
        fileToInclude = include.attrib['file']
        #Kill the nodes in the included file
        packageName = fileToInclude.split(' ')[1].split(')')[0]
        includeFileName = fileToInclude.split('/')[2]
        isLaunched = recursivekillLaunchFile(packageName, includeFileName)
    for node in nodes:
        #Kill each node
        if('ns' in node.attrib):
            ns = robot_namespace.attrib['default'] if robot_namespace is not None else node.attrib['ns']
            nodeName = ns + '/' + node.attrib['name']
        else:
            nodeName = node.attrib['name']  
        command = "rosnode kill {0}".format(nodeName)
        p = subprocess.Popen(command, shell=True)
        state = p.poll()
        if state is not None:
            return True
    
    
    return isLaunched

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
