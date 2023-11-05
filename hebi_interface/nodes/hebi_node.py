# !/usr/bin/env python

import rospy
import hebi
from time import sleep
import numpy as np
from snake_msgs.msg import JointCmd, StateFeedback
from std_msgs.msg import Header
import glob
import os
import rospkg

def HebiExec(msg, args):
    modules = args[0]
    jointCommand = args[1]
    log = args[2]
    feedback = args[3]
    sort_key = args[4]
    torque_limit = args[5]

    jointCommand.effort_limit_max = torque_limit*np.ones((modules.size, 1))
    jointCommand.position = sort_command(msg.Joint_Ang, sort_key)
    modules.send_command(jointCommand)

def robotModuleOrder(filepath, commsOrder):
    # Obtain the Order of modules on the physical snake
    file = open(moduleOrderFile, "r")
    moduleOrder = file.read().splitlines()[1:]
    file.close()

    if commsOrder is not None:
    # Create a sorting key to process joint commands
        sort_key = [moduleOrder.index(x) for x in commsOrder]
    else:
        sort_key = [moduleOrder.index(x) for x in np.sort(moduleOrder)]
    return sort_key

def sort_command(command, key):
    command_sorted = [command[key[i]] for i in range(len(key))]
    return command_sorted

def sendFeedback(pub, modules, sort_key):
    fbk = hebi.GroupFeedback(modules.size)
    fbk = modules.get_next_feedback()
    msg = StateFeedback()
    msg.header = Header(stamp=rospy.Time.now(),frame_id="cmd_ang")
    #print(fbk["position"])
    msg.Position = fbk.position[sort_key]
    msg.Velocity = fbk.velocity[sort_key]
    msg.Force = fbk.effort[sort_key]
    pub.publish(msg)


# Get a list of modules discovered by HEBI
for i in range(5):
    moduleList = hebi.Lookup()
    sleep(2)
    if moduleList is not None:
        break
print(moduleList)
num_modules = 0
for x in moduleList.entrylist:
    print(f'{x.family} | {x.name}')
    num_modules += 1
if num_modules==16:
    print("All Modules Detected")
else:
    raise Exception("All Modules not functioning")

modules = moduleList.get_group_from_family('*')

if modules.request_info() is not None:
    moduleNames = modules.request_info().name
else:
    moduleNames = None

rp = rospkg.RosPack()
moduleOrderFile = rp.get_path("hebi_interface") + "/data/" + rospy.get_param("/snake_type") + "moduleOrder.txt"
sort_key = robotModuleOrder(moduleOrderFile, moduleNames)

rospy.init_node("hebi_node")
print("Listening to Controller Commands")


# 500ms command lifetime. Controller should ideally publish at 100Hz (every 10ms)
cmd_lifetime = rospy.get_param('/Command_Lifetime', 500)
modules.command_lifetime = cmd_lifetime
torque_limit = rospy.get_param('/Max_Torque', 7.0)

jointCommand = hebi.GroupCommand(modules.size)

logging = rospy.get_param('/SnakeLog/log',"yes") # Always Log by default when running the Snake
filedir = rospy.get_param('/SnakeLog/filedir', None)
pathtoroot = '../' #HEBI starts logging in cwd which is a hidden .ros folder. Navigate back to ~/

if logging=='yes':
    if filedir is None:
        raise Exception("Snake Run : Logging is set to Yes but logging Directory not provided")
    print("Starting Log")
    modules.start_log(directory=pathtoroot+filedir)
    feedback = hebi.GroupFeedback(modules.size)

sub = rospy.Subscriber("JointAngles",JointCmd, HebiExec,(modules, jointCommand, logging, feedback, sort_key, torque_limit))
pub = rospy.Publisher("StateFeedback", StateFeedback, queue_size=64)
pub_rate = rospy.Rate(rospy.get_param("/Control_Frequency", 100))

while not(rospy.is_shutdown()):
    sendFeedback(pub, modules, sort_key)
    pub_rate.sleep()
rospy.spin()

if logging == 'yes':
    modules.stop_log()


