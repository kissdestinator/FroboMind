#!/usr/bin/env python

import asyncore, socket, time, sys
import roslib; roslib.load_manifest('fmSensors')
import rospy
from fmMsgs.msg import gtps

class GTPClient(asyncore.dispatcher):

    def __init__(self):
        rospy.init_node('gtps_node')

        host = rospy.get_param('~host', '127.0.0.1')
        port = rospy.get_param('~port', 15010)
        topic = rospy.get_param('~publisher_topic', 'gtps_position')
        self.senderID = rospy.get_param('~sender_id')

        print host, port, topic

        asyncore.dispatcher.__init__(self)
        try:
            self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connect( (host, port) )
        except socket.error(errorno, string):
            rospy.logerr("%s: %s" % (errorno, string))

        self.pub = rospy.Publisher('%s/%d' % (topic, self.senderID), gtps)

    def handle_read(self):
        if rospy.is_shutdown():
            self.close()
        else:
            data = self.recv(8192).strip().replace(';', '')
            rospy.loginfo(data)
            data = data.split(',')

            # Need to have at least 2 receivers
            if len(data) >= 12:
                if data[2] == '1':
                    if int(data[1]) == self.senderID:
                        msg = gtps()
                        msg.time = int(data[0])
                        msg.x = int(data[3])
                        msg.y = int(data[4])
                        msg.z = int(data[5])
                        receivers = []
                        distances = []
                        levels = []
                        i = 0
                        for field in data[6:]:
                            if i==0:
                                receivers.append(int(field))
                            elif i==1:
                                distances.append(int(field))
                            else:
                                levels.append(int(field))
                            i+=1
                            if i==3:
                                i=0
                        msg.receivers = receivers
                        msg.distances = distances
                        msg.levels = levels

                        self.pub.publish(msg)
                else:
                    rospy.logwarn("Received data was not valid.")
            else:
                rospy.logerr("Received data was in wrong format!")

    def handle_error(self):
        t, e, trace = sys.exc_info()
        rospy.logerr("%s: %s" % (t, e))

if __name__ == '__main__':
    try:
        GTPClient()
        asyncore.loop()
    except rospy.ROSInterruptException: pass


