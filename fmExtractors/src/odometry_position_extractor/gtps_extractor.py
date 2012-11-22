#!/usr/bin/env python
import roslib; roslib.load_manifest('fmExtractors')
import math, rospy, tf

from fmMsgs.msg import gtps
from nav_msgs.msg import Odometry

previous_message = None
previous_angle = 0
previous_time = None
prev_msg_for_angle = None


def handle_gtps_position(msg, senderID):
    global previous_message
    global previous_angle
    global previous_time
    global prev_msg_for_angle
    currentTime = rospy.Time.now()

    if previous_message:
        deltax = msg.x - previous_message.x
        deltay = msg.y - previous_message.y
        dt = (currentTime - previous_time).to_sec()
    else:
        deltax = 0
        deltay = 0
        dt = 1
        prev_msg_for_angle = msg

    previous_time = currentTime
    previous_message = msg

    distance = math.fabs(msg.x - prev_msg_for_angle.x) + math.fabs(msg.y - prev_msg_for_angle.y)

    if distance > 30:
        angle = math.atan2(msg.x - prev_msg_for_angle.x, msg.y - prev_msg_for_angle.y)
        
        prev_msg_for_angle = msg
        previous_angle = angle
    else:
        angle = previous_angle

    angle *= -1 #it works!
    rospy.loginfo(angle)

    rotationQuaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
    

    tfbr = tf.TransformBroadcaster()
    tfbr.sendTransform(
        (float(msg.x)/1000.0, float(msg.y)/1000.0, 0),
        rotationQuaternion,
        currentTime,
        'base_link',
        'odom')

    pub = rospy.Publisher('odom', Odometry)
    odom = Odometry()
    odom.header.stamp = currentTime 
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'

    odom.pose.pose.position.x = float(msg.x)/1000.0
    odom.pose.pose.position.y = float(msg.y)/1000.0
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation.x = rotationQuaternion[0]
    odom.pose.pose.orientation.y = rotationQuaternion[1]
    odom.pose.pose.orientation.z = rotationQuaternion[2]
    odom.pose.pose.orientation.w = rotationQuaternion[3]

    odom.pose.covariance = [    #we should do something with msg.levels to calculate better covariance
        1, 0, 0, 0, 0, 0, 
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 99999, 0, 0,
        0, 0, 0, 0, 99999, 0,
        0, 0, 0, 0, 0, 99999
    ]

    odom.twist.twist.linear.x = float(deltax)/(1000.0 * dt)
    odom.twist.twist.linear.y = float(deltay)/(1000.0 * dt)
    odom.twist.twist.linear.z = 0

    pub.publish(odom)

    

if __name__ == '__main__':
    rospy.init_node('gtps_tf_broadcaster')
    senderID = rospy.get_param('~sender_id')
    topic = rospy.get_param('~publisher_topic', 'gtps_position')
    rospy.Subscriber(
        '%s/%d' % (topic, senderID),
        gtps,
        handle_gtps_position,
        str(senderID))
    rospy.spin()
