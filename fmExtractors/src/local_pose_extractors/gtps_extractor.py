#!/usr/bin/env python
import roslib; roslib.load_manifest('fmExtractors')
import math, rospy, tf

from fmMsgs.msg import gtps

previous_message = None

def handle_gtps_position(msg, senderID):
	if previous_message:
		deltax = msg.x - previous_message.x
		deltay = msg.y - previous_message.y
	else:
		deltax = 0
		deltay = 0

	br = tf.TransformBroadcaster()
	br.sendTransform(
		(msg.x, msg.y, 0),
		tf.transformations.quaternion_from_euler(0, 0, math.atan2(deltax, deltay)),
		rospy.Time.now(),
		senderID,
		'world')

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
