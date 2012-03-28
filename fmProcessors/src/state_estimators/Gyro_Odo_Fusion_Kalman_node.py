#!/usr/bin/env python
import roslib; roslib.load_manifest('fmProcessors')
import rospy
from fmMsgs.msg import Vector3
def callback(data):
    rospy.loginfo(rospy.get_name()+"I heard %s",data.x)

def listener():
    rospy.init_node('Razor_Kalman')
    rospy.Subscriber("chatter", Vector3 , callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
