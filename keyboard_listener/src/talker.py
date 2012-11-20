#!/usr/bin/env python
import roslib; roslib.load_manifest('keyboard_listener')
import rospy
from std_msgs.msg import String
import termios, fcntl, sys, os
fd = sys.stdin.fileno()


keycode_r = 0x43 
keycode_l = 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)

oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)


def talker():
	pub = rospy.Publisher('key_pressed', String)
	rospy.init_node('talker')
	command_active=False
	#command='STOP'
	command=0
	publish=False
	try:
		while not rospy.is_shutdown():
			try:
				str = sys.stdin.read(1)
				if len(str)>0 and command_active:
					#print "aaa"
					command_active=False
					#command='STOP'
					command=0
					publish=True
				else:
					command_active=True
					if str == 'w':
						#command='FORWARD'
						command=1
						publish=True
					if str == 's':
						#command='BACKWARD'
						command=2
						publish=True
					if str == 'a':
						#command='LEFT'
						command=3
						publish=True
					if str == 'd':
						#command='RIGHT'
						command=4
						publish=True
				if publish:
					rospy.loginfo(command)
					pub.publish(String(command))


				# escape'\x1b' :

			
				#print "Got character", repr(str)
			except IOError: pass
	finally:
		termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
		fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)


if __name__ == '__main__':
	talker()
