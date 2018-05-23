#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped
import tf 

def main():
    '''uthai_control Publisher'''
    tf_listen = tf.TransformListener()
    pub_ssp = rospy.Publisher('uthai/spp', PolygonStamped, queue_size=3)
    pub_com = rospy.Publisher('uthai/com', PointStamped, queue_size=3)
    rospy.init_node('uthai_control')
    rate = rospy.Rate(10)  # 10hz

    spp_msg = PolygonStamped()
    # spp_msg.header = Header()
    spp_msg.header.frame_id = "r_foot_ft_link"
    com_msg = PointStamped()
    # com_msg.header = Header()
    com_msg.header.frame_id = "base_footprint"
    flag = 1
    while not rospy.is_shutdown():
        
        spp_msg.header.stamp = rospy.Time.now()
        com_msg.header.stamp = rospy.Time.now()
        spp_msg.polygon.points.append(Point32(0.07, 0.07, 0))
        spp_msg.polygon.points.append(Point32(0.07, -0.07, 0))
        spp_msg.polygon.points.append(Point32(-0.07, -0.07, 0))
        spp_msg.polygon.points.append(Point32(-0.07, 0.07, 0))
        
        if flag:
            com_msg.point.x += 0.005
        else:
            com_msg.point.x -= 0.005
        if com_msg.point.x > 0.1:
            flag = 0
        elif com_msg.point.x < -0.1:
            flag = 1
        
        # rospy.loginfo(spp_msg)
        # (trans,rot) = tf_listen.lookupTransformFull('/r_foot_ft_link','/base_footprint')
        (trans,rot) = tf_listen.lookupTransform('/r_foot_ft_link','/l_foot_ft_link', rospy.Time(0))
        # rospy.loginfo(trans)
        rospy.loginfo(rot)
        # rospy.loginfo(com_msg)
        pub_ssp.publish(spp_msg)
        pub_com.publish(com_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
