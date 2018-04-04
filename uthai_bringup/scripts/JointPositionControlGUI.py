#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from dynamixel_controllers.srv import *

LAP = rospy.Publisher(
    '/l_ankle_pitch_controller/command', Float64, queue_size=10)
LAR = rospy.Publisher(
    '/l_ankle_roll_controller/command', Float64, queue_size=10)
LHP = rospy.Publisher(
    '/l_hip_pitch_controller/command', Float64, queue_size=10)
LHR = rospy.Publisher('/l_hip_roll_controller/command', Float64, queue_size=10)
LHY = rospy.Publisher('/l_hip_yaw_controller/command', Float64, queue_size=10)
LKP = rospy.Publisher(
    '/l_knee_pitch_controller/command', Float64, queue_size=10)
RAP = rospy.Publisher(
    '/r_ankle_pitch_controller/command', Float64, queue_size=10)
RAR = rospy.Publisher(
    '/r_ankle_roll_controller/command', Float64, queue_size=10)
RHP = rospy.Publisher(
    '/r_hip_pitch_controller/command', Float64, queue_size=10)
RHR = rospy.Publisher('/r_hip_roll_controller/command', Float64, queue_size=10)
RHY = rospy.Publisher('/r_hip_yaw_controller/command', Float64, queue_size=10)
RKP = rospy.Publisher(
    '/r_knee_pitch_controller/command', Float64, queue_size=10)
LAPS = rospy.ServiceProxy('/l_ankle_pitch_controller/set_speed', SetSpeed)
LARS = rospy.ServiceProxy('/l_ankle_roll_controller/set_speed', SetSpeed)
LHPS = rospy.ServiceProxy('/l_hip_pitch_controller/set_speed', SetSpeed)
LHRS = rospy.ServiceProxy('/l_hip_roll_controller/set_speed', SetSpeed)
LHYS = rospy.ServiceProxy('/l_hip_yaw_controller/set_speed', SetSpeed)
LKPS = rospy.ServiceProxy('/l_knee_pitch_controller/set_speed', SetSpeed)
RAPS = rospy.ServiceProxy('/r_ankle_pitch_controller/set_speed', SetSpeed)
RARS = rospy.ServiceProxy('/r_ankle_roll_controller/set_speed', SetSpeed)
RHPS = rospy.ServiceProxy('/r_hip_pitch_controller/set_speed', SetSpeed)
RHRS = rospy.ServiceProxy('/r_hip_roll_controller/set_speed', SetSpeed)
RHYS = rospy.ServiceProxy('/r_hip_yaw_controller/set_speed', SetSpeed)
RKPS = rospy.ServiceProxy('/r_knee_pitch_controller/set_speed', SetSpeed)


def JointStateCallback(data):
    # ["r_hip_yaw_joint", "r_hip_roll_joint", "r_hip_pitch_joint", "r_knee_pitch_joint", "r_ankle_pitch_joint","r_ankle_roll_joint",
    # "l_hip_yaw_joint", "l_hip_roll_joint", "l_hip_pitch_joint", "l_knee_pitch_joint", "l_ankle_pitch_joint", "l_ankle_roll_joint"]

    # rospy.loginfo("I heard %s", data.position[0])
    RHY.publish(data.position[0])
    RHR.publish(data.position[1])
    RHP.publish(data.position[2])
    RKP.publish(data.position[3])
    RAP.publish(data.position[4])
    RAR.publish(data.position[5])
    LHY.publish(data.position[6])
    LHR.publish(data.position[7])
    LHP.publish(data.position[8])
    LKP.publish(data.position[9])
    LAP.publish(data.position[10])
    LAR.publish(data.position[11])
    if True:
        RHYS(data.velocity[0])
        RHRS(data.velocity[1])
        RHPS(data.velocity[2])
        RKPS(data.velocity[3])
        RAPS(data.velocity[4])
        RARS(data.velocity[5])
        LHYS(data.velocity[6])
        LHRS(data.velocity[7])
        LHPS(data.velocity[8])
        LKPS(data.velocity[9])
        LAPS(data.velocity[10])
        LARS(data.velocity[11])


def main():
    rospy.init_node('GUI_Control')

    rospy.Subscriber("/joint_states", JointState, JointStateCallback)
    rospy.spin()

if __name__ == '__main__':
    main()
