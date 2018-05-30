#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from tf import transformations

def main():
    pub = rospy.Publisher("/uthai/sensor/imu", Imu, queue_size=10)
    rospy.init_node('test_imu_pub')
    rate = rospy.Rate(7)
    while not rospy.is_shutdown():
        msg = Imu()
        msg.header.frame_id = "imu_link"
        msg.header.stamp = rospy.Time.now()
        quaternion = transformations.quaternion_from_euler(3.14/2,0.0,0.0)
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        msg.angular_velocity.x = 0.4
        msg.angular_velocity.y = 1.0
        msg.angular_velocity.z = 0.6
        msg.linear_acceleration.x = 0.02
        msg.linear_acceleration.y = -0.04
        msg.linear_acceleration.z = -0.98
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
