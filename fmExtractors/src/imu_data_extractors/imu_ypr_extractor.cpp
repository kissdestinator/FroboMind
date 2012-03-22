#include "ros/ros.h"
#include <string.h>
#include "fmMsgs/serial.h"
#include "fmMsgs/ypr.h"
#include "fmMsgs/accelerometer.h"
#include "fmMsgs/magnetometer.h"
#include <cmath>
#include "nr3.h"

#define DEG2RAD M_PI/180.0
#define RAD2DEG 180.0/M_PI

using namespace std;
float accel[3];
float mag[3];

void accelCallback(fmMsgs::accelerometer accelMsg)
{
	accel[0] = accelMsg.y;
	accel[1] = accelMsg.x;
	accel[2] = accelMsg.z;
}
void magCallback(fmMsgs::magnetometer magMsg)
{
	mag[0] = magMsg.x;
	mag[1] = magMsg.y;
	mag[2] = magMsg.z;
}


float Compass_Heading(float roll, float pitch)
{
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // Tilt compensated magnetic field X
  mag_x = mag[0]*cos_pitch + mag[1]*sin_roll*sin_pitch + mag[2]*cos_roll*sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = mag[1]*cos_roll - mag[2]*sin_roll;
  // Magnetic Heading
  return atan2(-mag_y, mag_x);
}

int main(int argc, char **argv)
{
	std::string publisher_topic;
	std::string subscriber_topic_accel;
	std::string subscriber_topic_mag;
	int update_frequency;
	ros::init(argc, argv, "imu_ypr_extractor");


	// Nodehandlers
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

  	/* read parameters from ros parameter server if available otherwise use default values */
 	 nh.param<std::string> ("publisher_topic", publisher_topic, "ypr"); //Specify the publisher name
 	 nh.param<std::string> ("subscriber_topic_accel", subscriber_topic_accel, "/fmSensors/Accelerometer"); //Specify the subscriber name for the accelerometer
 	 nh.param<std::string> ("subscriber_topic_mag", subscriber_topic_mag, "/fmSensors/Magnetometer"); //Specify the subscriber name for the magnetometer
	 nh.param<int> ("update_frequency", update_frequency, 50); //Specify the update frequency

	ros::Subscriber accelSub = n.subscribe(subscriber_topic_accel, 1, accelCallback);
	ros::Subscriber magSub = n.subscribe(subscriber_topic_mag, 1, magCallback);
	ros::Publisher	yprPub = n.advertise<fmMsgs::ypr>(publisher_topic, 1);

	ros::Rate loop_rate(update_frequency);

	fmMsgs::ypr ypr;

	float pitch, roll, yaw, mag_x, mag_y;

	while (ros::ok())
	{
		 //Get pitch value from acceletometer
		pitch = -atan(accel[0]/sqrt(accel[1]*accel[1]+accel[2]*accel[2])) ;
		 //Get roll value from accelerometer 
		roll = atan2(accel[1],accel[2]) * -1;
		
		/*
		 //Get pitch value from acceletometer
		pitch = -atan2(accel[0], sqrt(accel[1]*accel[1]+accel[2]*accel[2])) ;
		 //Get roll value from accelerometer 
		roll = atan(accel[1]/accel[2]);
		 //Calculate the mags
		//mag_x = mag[0]*cos(pitch) + mag[1] * sin(roll) * sin(pitch) + mag[2] * cos(roll) * sin(pitch);
		//mag_y = mag[1]*cos(roll) - mag[2] * sin(roll);"
		float temp1 = (mag[2]*sin(roll)-mag[1]*cos(roll));
		float temp2 = ( mag[0]*cos(pitch)+mag[1]*sin(pitch)*sin(roll)+mag[2]*sin(pitch)*cos(roll));
		yaw = atan(temp1/temp2);
		ROS_INFO("Yaw: %f, Pitch: %f, Roll: %f, X: %f, Y: %f, Z: %f, temp1 = %f, temp2 = %f", yaw * RAD2DEG, pitch, roll, mag[0], mag[1], mag[2], temp1, temp2);
		 //Calculate yaw
		//yaw = atan2(-mag_y, mag_x);	
		*/

		yaw = atan2((mag[2]*sin(roll)-mag[1]*cos(roll)),( mag[0]*cos(pitch)+mag[1]*sin(pitch)*sin(roll)+mag[2]*sin(pitch)*cos(roll)));
		//yaw = Compass_Heading(roll,pitch);

		ypr.pitch = pitch * RAD2DEG;
		ypr.roll = roll* RAD2DEG;
		ypr.yaw = yaw* RAD2DEG;			
		
		yprPub.publish(ypr);

		loop_rate.sleep();

		ros::spinOnce();
	}

	return 0;
}
