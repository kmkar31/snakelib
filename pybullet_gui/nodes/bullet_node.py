#!/usr/bin/env python

# Listens to the JointCmd topic and executes a change in the joint state on the GUI

import rospy
from snake_msgs.msg import JointCmd
from snake_msgs.msg import StateFeedback
import pybullet as p
import pybullet_data
import rospkg
import numpy as np
from std_msgs.msg import Header

counter = 0

def load_terrain():
    rp = rospkg.RosPack()
    '''
    terrainPath = rp.get_path("model") + "/terrain/plate1.urdf"
    p.loadURDF(terrainPath)
    terrainPath2 = rp.get_path("model") + "/terrain/plate2.urdf"
    p.loadURDF(terrainPath2)
    '''
    terrainPath3 = rp.get_path("model") + "/terrain/plate3.urdf"
    p.loadURDF(terrainPath3)
    terrainPath4 = rp.get_path("model") + "/terrain/plate4.urdf"
    p.loadURDF(terrainPath4)
    terrainPath5 = rp.get_path("model") + "/terrain/plate5.urdf"
    #p.loadURDF(terrainPath5)
    terrainPath3 = rp.get_path("model") + "/terrain/flat_plane15_15.urdf"
    #p.loadURDF(terrainPath3)
    #'''
    p.loadURDF("plane.urdf")
    '''
    for i in range(20):
        terrainPath = rp.get_path("model") + "/terrain/pegboard/pegboard" + str(i+1) + '.urdf'
        p.loadURDF(terrainPath)
    '''
    print("Terrain Loaded")

def callback(msg,args):
    snake_Id = args[0]
    joint_idx = args[1]
    state_pub = args[2]
    torque_max = args[3]

    p.setJointMotorControlArray(bodyUniqueId = snake_Id,jointIndices = joint_idx,controlMode = p.POSITION_CONTROL,
                                targetPositions=msg.Joint_Ang,targetVelocities=np.zeros(len(joint_idx)),forces=torque_max*np.ones(len(joint_idx)))
    
    global counter
    
    '''
    if counter < 3000:
        print("Applying External Forcing")
        #for i in range(16):
            #p.applyExternalForce(snake_Id,joint_idx[6], [0,(-1)**(i+1)*35,0],[0,0,0],p.WORLD_FRAME)
        p.applyExternalForce(snake_Id,joint_idx[0], [-60,0,0],[0,0,0],p.WORLD_FRAME)
        p.applyExternalForce(snake_Id, joint_idx[-1], [60,0,0],[0,0,0],p.WORLD_FRAME)
        #p.applyExternalForce(snake_Id, joint_idx[9], [20,0,0],[0,0,0],p.LINK_FRAME)
        #p.applyExternalTorque(snake_Id,joint_idx[5], [0,0,8],p.WORLD_FRAME)
        #p.applyExternalTorque(snake_Id,joint_idx[-5], [0,0,-8],p.WORLD_FRAME)
        counter = counter + 1
    if counter==3000:
        print("Ending External Forcing")
        counter = counter + 1
    '''
    p.stepSimulation()

    pub_msg = StateFeedback()
    pub_msg.cmd_name = "State Feedback"

    jointStates = p.getJointStates(snake_Id, joint_idx)
    Positions = np.zeros((len(jointStates)))
    Velocities = np.zeros((len(jointStates)))
    Torques= np.zeros((len(jointStates)))
    AppTorques = np.zeros((len(jointStates)))
    for i in range(len(jointStates)):
        Positions[i] = jointStates[i][0]
        Velocities[i] = jointStates[i][1]
        #print(jointStates[i][2][3:])
        Torques[i] = jointStates[i][2][5] # FOR RSNAKE - THIS IS INDEX 3 or 4 because axis systems are wrong on CAD
        AppTorques[i] = jointStates[i][3]
    #print("Applied Torque",np.linalg.norm(AppTorques))
   
    pub_msg.Force = Torques
    pub_msg.Position = Positions
    pub_msg.Velocity = Velocities
    pub_msg.header = Header(stamp=rospy.Time.now(),frame_id="state")
    #print(*Torques, sep=',')
    try:
        state_pub.publish(pub_msg)
    except:
        print("Check the published message for datatype mismatches")

def init_bullet():
    # Load Environment
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    print("PyBullet Physics Environment Ready")
    
    load_terrain()
    startPos = [0,0,0.1]
    startOrientation = p.getQuaternionFromEuler([0,90,0])
    rp = rospkg.RosPack()
    # Load Snake
    snake_type = rospy.get_param('/snake_type')
    path = rp.get_path("model") + "/" + snake_type + "/" + snake_type + ".urdf"
    snake_Id = p.loadURDF(path,startPos, startOrientation)
    joint_idx = parse_joint_info(snake_Id)
    print("Snake Robot Loaded")

    for idx in joint_idx:
        p.enableJointForceTorqueSensor(snake_Id, idx, 1)
    
    return [snake_Id,joint_idx]

def parse_joint_info(snake_Id): # The URDF file has fixed joints defined to describe the joints between links. This functions extracts the revolute joints
    joint_idx = []
    for i in range(p.getNumJoints(snake_Id)):
        joint_info = p.getJointInfo(snake_Id,i)
        if joint_info[2] == p.JOINT_REVOLUTE: # JOINT_REVOLUTE shows up as index 0 in joint info ; prismatic as 1 and fixed as 4
            joint_idx.append(i)
    print(len(joint_idx))
    return joint_idx

def process_feedback(forcevec):
    N = rospy.get_param("/N")
    torques = np.zeros((N,1))
    torquevec = forcevec[:,3:]
    for i in range(N):
        torques[i] = torquevec[i,2] # Mz component
    print(','.join(str(torques)))
    return torques

# Initialize node
counter = 0
rospy.init_node("bullet_node")
[snake_Id,joint_idx] = init_bullet()
torque_max = rospy.get_param('/Max_Torque', 7.0)

p.setRealTimeSimulation(0)
if rospy.get_param("/SimLog/log","no") == "yes":
    filepath = rospy.get_param("/SimLog/filedir") + "/" + "run1.mp4" #rospy.get_param("/gait_type","linear_progression") + ".mp4"
    p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,filepath)

# Publisher that publishes the current Joint Angles of the snake
state_pub = rospy.Publisher("StateFeedback",StateFeedback,queue_size=32)
print("Pybullet Ready to publish Joint States")

# Subscriber that listens to messages from gait_control and executes them on the GUI
sub = rospy.Subscriber("JointAngles",JointCmd,callback,(snake_Id,joint_idx,state_pub, torque_max))
print("Pybullet Listening for Joint Angle commands")
rospy.spin()

