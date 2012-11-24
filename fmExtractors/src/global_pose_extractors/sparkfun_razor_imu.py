#!/usr/bin/env python
import roslib; roslib.load_manifest('fmExtractors')
import rospy
from tf import transformations
from fmMsgs.msg import gyroscope, accelerometer
from sensor_msgs.msg import Imu

class razorIMU():
	filter = 0.1
	gyro = None
	accel = None
	dt = 0
	accel_calibrated = False
	gyro_calibrated = False
	samples_accel = 0
	samples_gyro = 0
	offset_accel_x = 0
	offset_accel_y = 0
	offset_accel_z = 0
	offset_gyro_x = 0
	offset_gyro_y = 0
	offset_gyro_z = 0

	roll = 0
	pitch = 0
	yaw = 0

	def __init__(self):
		rospy.init_node('razorIMU')
		self.pub = rospy.Publisher('/fmExtractors/razorIMU', Imu)

		rospy.Subscriber("/fmSensors/Gyroscope", gyroscope , self.gyro_callback)
		rospy.Subscriber("/fmSensors/Accelerometer", accelerometer , self.accel_callback)

		rospy.spin()

	def callibrate_accel(self):
		self.offset_accel_x += self.accel.x
		self.offset_accel_y += self.accel.y
		self.offset_accel_z += self.accel.z

		if self.samples_accel == 127:
			self.offset_accel_x /= 128
			self.offset_accel_y /= 128
			self.offset_accel_z /= 128
			self.accel_calibrated = True

			rospy.loginfo("SparkFun Razor IMU Accelerometer calibration finished.")

		self.samples_accel += 1

	def callibrate_gyro(self):
		self.offset_gyro_x += self.gyro.x
		self.offset_gyro_y += self.gyro.y
		self.offset_gyro_z += self.gyro.z

		if self.samples_gyro == 127:
			self.offset_gyro_x /= 128
			self.offset_gyro_y /= 128
			self.offset_gyro_z /= 128
			self.gyro_calibrated = True

			rospy.loginfo("SparkFun Razor IMU Gyroscope calibration finished.")

		self.samples_gyro += 1

	def gyro_callback(self, data):
		self.dt = data.header.time.to_sec() - self.gyro.header.time.to_sec()
		self.gyro = data

		if self.gyro_calibrated:
			self.gyro.x -= self.offset_gyro_x
			self.gyro.y -= self.offset_gyro_y
			self.gyro.z -= self.offset_gyro_z
		else:
			self.callibrate_gyro()

	def accel_callback(self, data):
		self.accel = data

		if self.accel_calibrated:
			self.accel.x -= self.offset_accel_x
			self.accel.y -= self.offset_accel_y
			self.accel.z -= self.offset_accel_z

			if self.gyro_calibrated:
				self.roll = (1-self.filter)*(self.roll + self.gyro.x*self.dt) + self.filter*self.accel.x
				self.pitch = (1-self.filter)*(self.pitch + self.gyro.y*self.dt) + self.filter*self.accel.y
				self.yaw = (1-self.filter)*(self.yaw + self.gyro.z*self.dt) + self.filter*self.accel.z

				quaternion = transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
				msg = Imu()
				msg.orientation.x = quaternion[0]
				msg.orientation.y = quaternion[1]
				msg.orientation.z = quaternion[2]
				msg.orientation.w = quaternion[3]

				msg.angular_velocity.x = self.gyro.x
				msg.angular_velocity.y = self.gyro.y
				msg.angular_velocity.z = self.gyro.z

				msg.linear_acceleration.x = self.accel.x
				msg.linear_acceleration.y = self.accel.y
				msg.linear_acceleration.z = self.accel.z
		else:
			self.callibrate_accel()



if __name__ == '__main__':

    razorIMU()
