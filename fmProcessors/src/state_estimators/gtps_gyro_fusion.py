#!/usr/bin/env python
import roslib; roslib.load_manifest('fmProcessors')
import rospy, tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from fmMsgs.msg import gyroscope
from math import *

gyroOffset = -0.01492
odo = None
cmd = Twist()
lastInterruptTime = 0
gyroAngVel = 0
heading = 0

def fusion_calc(event):
    global odo, pub, lastInterruptTime, gyroAngVel, heading, cmd
    gyroAngVel = drift_correction(gyroAngVel)

    #rotationQuaternion = tf.transformations.quaternion_from_euler(0, 0, -1*((x.value[0][0]-pi) % (pi * 2)))

    if odo:
        if cmd.angular.z == 0:
            heading = 0
        else:
            heading += gyroAngVel

        x,y,z = tf.transformations.euler_from_quaternion([odo.pose.pose.orientation.x, odo.pose.pose.orientation.y, odo.pose.pose.orientation.z, odo.pose.pose.orientation.w])
        z += heading*pi/180.0

        if cmd.linear.x < 0 or cmd.linear.y < 0:
            z *=-1

        rotationQuaternion = tf.transformations.quaternion_from_euler(x,y,z)

        tfbr = tf.TransformBroadcaster()
        tfbr.sendTransform(
            (odo.pose.pose.position.x, odo.pose.pose.position.y, odo.pose.pose.position.z),
            rotationQuaternion,
            rospy.Time.now(),
            'base_link',
            'odom')

        odo.pose.pose.orientation.x = rotationQuaternion[0]
        odo.pose.pose.orientation.y = rotationQuaternion[1]
        odo.pose.pose.orientation.z = rotationQuaternion[2]
        odo.pose.pose.orientation.w = rotationQuaternion[3]

        pub.publish(odo)

    lastInterruptTime = event.current_real.to_sec()

def drift_correction(gyroAng):
    if (gyroAng < 0.004) and (gyroAng > -0.004):
        return 0
    return gyroAng

def gyro_callback(data):
    global gyroAngVel, gyroOffset
    gyroAngVel = data.z - gyroOffset

def odo_callback(data):
    global odo
    odo = data

def cmd_callback(data):
    global cmd
    cmd = data

def gtps_gyro_fusion():
    rospy.init_node('gtps_gyro_fusion')
    rospy.Subscriber("/fmSensors/Gyroscope", gyroscope , gyro_callback)
    rospy.Subscriber("/cmd_vel", Twist , cmd_callback)
    rospy.Subscriber("/odom", Odometry , odo_callback)
    rospy.Timer(rospy.Duration(0.02), fusion_calc, oneshot = False)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/odom/fusion', Odometry)

    gtps_gyro_fusion()