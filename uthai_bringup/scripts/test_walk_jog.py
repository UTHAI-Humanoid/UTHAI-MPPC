#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

pi = 3.141
tim = 2.5
count = 1
q_leg_R = [[0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0],
           [0, 0, -0.5, 1.0, -0.5, 0],
           #    [-0.0649,0.0195, 0.8209,0.9682,0.0034,0.0007],
           #    [0.0058, 0.0620, -0.6297, 1.2826, -0.6357, -0.1816],
           #    #    [0, 0, -0.6283, 1.2566, -0.6283, 0],
           #    [0.0047, 0.0479, -0.6256, 1.2482, -0.6289, 0.0005],
           #    #    [0, 0, -0.6283, 1.2566, -0.6283, 0],
           #    [0.0058, 0.0620, -0.6297, 1.2826, -0.6357, -0.1816],
           #    [0, 0, -0.6283, 1.2566, -0.6283, 0]
           ]

q_leg_L = [[0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0],
           [0, 0, -0.5, 1.0, -0.5, 0],
           #    [ -0.0208,-0.1705,-0.4181,1.4445,-1.0018,0.1570],
           #    [-0.0054, -0.0559, -0.6258, 1.2484, -0.6289, -0.0006],
           #    #    [0, 0, -0.6283, 1.2566, -0.6283, 0],
           #    [-0.0051, -0.0568, -0.6298, 1.2856, -0.6369, 0.1551],
           #    #    [0, 0, -0.6283, 1.2566, -0.6283, 0],
           #    [-0.0054, -0.0559, -0.6258, 1.2484, -0.6289, -0.0006],
           #    [0, 0, -0.6283, 1.2566, -0.6283, 0]
           ]

pub = rospy.Publisher('/joint_states', JointState, queue_size=10)


def ActionCallback(msg):
    global count
    # ["r_hip_yaw_joint", "r_hip_roll_joint", "r_hip_pitch_joint", "r_knee_pitch_joint", "r_ankle_pitch_joint","r_ankle_roll_joint",
    # "l_hip_yaw_joint", "l_hip_roll_joint", "l_hip_pitch_joint", "l_knee_pitch_joint", "l_ankle_pitch_joint", "l_ankle_roll_joint"]

    # rospy.loginfo("I heard %s", q_leg_R[count] + q_leg_L[count])

    joint_msg = JointState()
    for joints, joints_vel in zip(q_leg_R[count] + q_leg_L[count], [abs(x - y) / tim for x, y in zip(q_leg_R[count] + q_leg_L[count], q_leg_R[count - 1] + q_leg_L[count - 1])]):
        joint_msg.position.append(joints)
        if joints_vel == 0:
            joints_vel = 0.2
        joint_msg.velocity.append(joints_vel)
    pub.publish(joint_msg)
    rospy.loginfo("I heard %s", joint_msg)
    count = count + 1
    if count > 2:
        count = 0


def main():
    rospy.init_node('actoinz')

    rospy.Subscriber("/actionz", Empty, ActionCallback)

    rospy.spin()


if __name__ == '__main__':
    main()
