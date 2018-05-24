#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


distorque = rospy.Publisher('uthai/torque_enable', Bool, queue_size=10)

def joycb(msg):
    if msg.buttons[4]:
        torque = Bool()
        if msg.buttons[6]:
            torque.data = False
        else:
            torque.data = True
        distorque.publish(torque)

        rospy.loginfo(msg.buttons[4]&msg.buttons[6])

def main():
    rospy.init_node("joy_tele")
    joy = rospy.Subscriber('joy', Joy, joycb)
    rospy.spin()

if __name__ == '__main__':
    main()