#!/usr/bin/env python

import rospy
from sensor_msgs.msg import String


def main():
    rospy.init_node("uthai_mbed")
    rospy.loginfo("ROS Serial to UTHAI Robot")

if __name__ == '__main__':
    main()
