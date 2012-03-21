#include "ros/ros.h"
#include <string.h>
#include "fmMsgs/serial.h"
#include "fmMsgs/ypr.h"
#include "fmMsgs/accelerometer.h"
#include "fmMsgs/magnetometer.h"
#include <cmath>

#define DEG2RAD M_PI/180.0
#define RAD2DEG 180.0/M_PI

using namespace std;
float accel[3];
float mag[3];

void accelCallback(fmMsgs::accelerometer accelMsg)
{
	accel[0] = accelMsg.x;
	accel[1] = accelMsg.y;
	accel[2] = accelMsg.z;
}
void magCallback(fmMsgs::magnetometer magMsg)
{
	mag[0] = magMsg.x;
	mag[1] = magMsg.y;
	mag[2] = magMsg.z;
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

	ros::Subscriber accelSub = n.subscribe(subscriber_topic_accel, 1, magCallback);
	ros::Subscriber magSub = n.subscribe(subscriber_topic_mag, 1, accelCallback);
	ros::Publisher	yprPub = n.advertise<fmMsgs::ypr>(publisher_topic, 1);

	ros::Rate loop_rate(update_frequency);

	fmMsgs::ypr ypr;

	float pitch, roll, yaw, mag_x, mag_y;

	while (ros::ok())
	{
		 //Get pitch value from acceletometer
		pitch = -atan2(accel[0], sqrt(accel[1]*accel[1]+accel[2]*accel[2])) - M_PI;
		float temp = accel[1]/accel[2];
		ROS_INFO("%f", temp);
		 //Get roll value from accelerometer 
		roll = atan(accel[1]/accel[2]) - M_PI;
		
		/*
		if(accel[1] > 0 && accel[0] < 0.5 && accel[0] > -0.5)
			roll = abs(atan(accel[0]/accel[1]));		
		else if(accel[0] > 0 && accel[1] < 0.5 && accel[1] > -0.5)
			roll = abs((atan(accel[1]/accel[0]) + (M_PI/2)));
		else if(accel[1] < 0 && accel[0] < 0.5 && accel[0] > -0.5)
			roll = abs((atan(accel[0]/accel[1]) + (M_PI)));
		else if(accel[0] < 0 && accel[1] < 0.5 && accel[1] > -0.5)
			roll = abs((atan(accel[1]/accel[0]) + (M_PI * 1.5)));
		*/

		
		 //Calculate the mags
		mag_x = mag[0]*cos(pitch) + mag[1] * sin(roll) * sin(pitch) + mag[2] * cos(roll) * sin(pitch);
		mag_y = mag[1]*cos(roll) - mag[2] * sin(roll);

		 //Calculate yaw
		yaw = atan2(-mag_y, mag_x);	

		ypr.pitch = pitch;
		ypr.roll = roll;
		ypr.yaw = yaw;			
		
		yprPub.publish(ypr);

		loop_rate.sleep();

		ros::spinOnce();
	}

	return 0;
}
