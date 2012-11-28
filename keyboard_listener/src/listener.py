#!/usr/bin/env python
import roslib; roslib.load_manifest('keyboard_listener')
import rospy, tf
from std_msgs.msg import String, Header
from fmMsgs.msg import motor_power

def callback(data):
	motorPub = rospy.Publisher('/fmControllers/motor_power', motor_power)
	motor_power_msg = motor_power()
	rospy.loginfo(rospy.get_name() + "Key pressed: %s", data.data)
	if data.data == "FORWARD":
		rospy.loginfo("Kurva")
		motor_power_msg.header = Header()
		motor_power_msg.power_right = 0.2
		motor_power_msg.power_left = 0.2
		motorPub.publish(motor_power_msg)

	if data.data == "STOP":
		rospy.loginfo("Kurva")
		motor_power_msg.header = Header()
		motor_power_msg.power_right = 0
		motor_power_msg.power_left = 0
		motorPub.publish(motor_power_msg)

def listener():
	


	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("key_pressed", String, callback)
	rospy.spin()
if __name__ == '__main__':
	listener()


#define _FREQUENCE_ 500
#define _MAX_MESSAGES_ 1
#define _TOPIC_MOTOR_ "/fmControllers/motor_power"
#define _MAX_MESSAGES_ 1

