# !/usr/bin/env python

# Node to send user input commands to modify commands
import rospy
from snake_msgs.msg import HumIntCmd
from curtsies import Input


def parse():
    return rospy.get_param("/keyboard")

def process_cmd(key,human):
    print("Command key : " + key)
    if key in rospy.get_param("/keyboard"):
        msg = HumIntCmd()
        msg.cmd_name = "GaitChange"
        action = key_bindings[key]
        idx = action.find('reverse')
        if idx == -1:
            msg.action = action
            msg.invert = 1
        else:
            msg.action = action[:idx-1]
            msg.invert = -1
        human.publish(msg)
    

# Initialize node
rospy.init_node("Human")

# Create publisher
human = rospy.Publisher("keyboard_cmd",HumIntCmd,queue_size=32)
rate = rospy.Rate(80)

# import key-bindings from human_operator/param

key_bindings = parse()
print(key_bindings)
while not(rospy.is_shutdown()):
    with Input(keynames='curses') as input_generator:
        for e in input_generator:
            if e in [u'<ESC>',u'<Ctrl-c>']:
                print("Terminating Keyboard-teleop")
                break
            else:
                process_cmd(e,human)