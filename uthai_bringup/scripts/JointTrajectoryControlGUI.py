#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

UTHAI_LEG = rospy.Publisher(
    '/uthai_leg_controller/command', JointTrajectory, queue_size=10)


def JointStateCallback(data):
    traject = JointTrajectory()
    traject.joint_names = [
        "r_hip_yaw_joint", "r_hip_roll_joint", "r_hip_pitch_joint", "r_knee_pitch_joint", "r_ankle_pitch_joint",
                            "r_ankle_roll_joint", "l_hip_yaw_joint", "l_hip_roll_joint", "l_hip_pitch_joint", "l_knee_pitch_joint", "l_ankle_pitch_joint", "l_ankle_roll_joint"]

    traject_points = JointTrajectoryPoint()
    traject.points = list()
    for i in range(12):
        traject_points.positions = [data.position[i]]
        traject_points.velocities = [0.5]
        traject.points.append(traject_points)
        # traject.points.positions.append(data.position[i])
        # traject.points.velocities.append(0.5)

    UTHAI_LEG.publish(traject)


def main():
    rospy.init_node('GUI_Control')

    rospy.Subscriber("/joint_states", JointState, JointStateCallback)
    rospy.spin()

if __name__ == '__main__':
    main()
