#!/usr/bin/env python
import roslib; roslib.load_manifest('fmExtractors')
import math, rospy, tf
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header

def handle_convert(data):
    pub = rospy.Publisher('/navigation_vel', TwistStamped)
    m = TwistStamped()
    m.twist = data
    m.header = Header()
    m.header.stamp = rospy.Time.now()
    pub.publish(m)

if __name__ == '__main__':
    rospy.init_node('twiststamped_to_twist')
    rospy.Subscriber(
        '/cmd_vel',
        Twist,
        handle_convert)
    rospy.spin()
