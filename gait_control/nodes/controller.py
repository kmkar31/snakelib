#!/usr/bin/env python

import rospy
from snake_msgs.msg import JointCmd, HumIntCmd, StateFeedback
from src.snake import Snake
from std_msgs.msg import Header
import time
import rospkg
import os

def process_cmd(publisher, t, angle=[]):
    msg = JointCmd()
    msg.cmd_name = "Joint Angle SET"
    msg.param_name = "Joint Angles"
    msg.header = Header(stamp=rospy.Time.now(),frame_id="cmd_ang")
    if len(angle)==0:
        ang = snake.update(t)
        msg.Joint_Ang = ang
    else:
        msg.Joint_Ang = angle
    publisher.publish(msg)


def process_feedback(feedback, args):
    pub = args[0]
    snake = args[1]
    t = (rospy.Time.now() - args[2]).to_sec()

    angles = snake.comply(feedback.Force, t)
    msg = JointCmd()
    msg.cmd_name = "Joint Angle SET"
    msg.param_name = "Joint Angles"
    msg.header = Header(stamp=rospy.Time.now(),frame_id="cmd_ang")
    msg.Joint_Ang = angles
    pub.publish(msg)
    log(snake, args[2], t)

def log(snake, runStart, t):
    filename = "../" + rospy.get_param("/InternalLog/filedir") + "/" + str(runStart) + ".csv"
    f = open(filename, 'a+')
    if os.path.getsize(filename) == 0:
        f.write("Time," + ','.join(list(snake.sigma_o.keys())) + "," + ','.join(list(snake.sigma_d.keys())) + ",Shape Force" + "\n")
    nom = list(snake.sigma_o.values()) 
    des = list(snake.sigma_d.values())
    f.write(str(t) + ",")
    for i in range(len(nom)):
        f.write(str(nom[i])+ ",")
    for i in range(len(des)):
        f.write(str(des[i])+",")
    f.write(str(snake.shapeForce[0][0]) + "\n")
    f.close()


def humanOrder(msg, args):
    snake = args[0]
    pub = args[1]
    NewGait = msg.action
    invert = msg.invert
    if snake.gaitType == NewGait and snake.invert == invert:
        print("Commanded Gait already executing")
        return
    else:
        print("Commanding transition to", NewGait)
        snake.transition(NewGait, invert)
        process_cmd(pub,0,snake.resetJoints())


rospy.init_node("ControllerBackbone")

JointPublisher = rospy.Publisher("JointAngles",JointCmd,queue_size=64)
print("Ready to Publish Joint Commands")

ctrl_freq = rospy.get_param("/Control_Frequency", 100)
rate = rospy.Rate(ctrl_freq)
gait = rospy.get_param("/gait_type")
snake_type = rospy.get_param("/snake_type")
snake = Snake(gait, snake_type)
StartTime = rospy.Time.now()

HumanListener = rospy.Subscriber("keyboard_cmd",HumIntCmd, humanOrder,(snake,JointPublisher))
print("Listening to Human Gait Commands")

Comply = rospy.get_param("/comply")
if Comply:
    FeedbackListener = rospy.Subscriber("StateFeedback", StateFeedback, process_feedback, (JointPublisher, snake, StartTime))
    print("Listening to Torque Feedback")
    #process_cmd(JointPublisher,0,snake.resetJoints())


while not(rospy.is_shutdown()):
    curr_time = rospy.Time.now()
    duration = (curr_time-StartTime).to_sec()
    if not Comply:
        process_cmd(JointPublisher, duration)
    rate.sleep()

rospy.spin()