#include "ros/ros.h"
#include <string.h>
#include "fmMsgs/serial.h"
#include "fmMsgs/ypr.h"

using namespace std;

//fmMsgs::serial serial;
fmMsgs::ypr ypr;

//#YPR=-26.31,-5.71,94.98
void yprCallback(fmMsgs::serial serialInput)
{
	//serial=serialInput;
	int startPos = 5;
	int nextComma = serialInput.data.find(',');

	
	ypr.yaw = atof(serialInput.data.substr(startPos,nextComma).c_str());
	
	startPos = nextComma+1;
	nextComma = serialInput.data.find(',',startPos);
	ypr.pitch = atof(serialInput.data.substr(startPos,nextComma).c_str()); 

	startPos = nextComma+1;
	ypr.roll = atof(serialInput.data.substr(startPos,serialInput.data.size()-1).c_str()); 
}

int main(int argc, char **argv)
{
	std::string publisher_topic;
	std::string subscriber_topic;
	int update_frequency;
	ros::init(argc, argv, "imu_ypr_extractor");


	// Nodehandlers
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

  	/* read parameters from ros parameter server if available otherwise use default values */
 	 nh.param<std::string> ("publisher_topic", publisher_topic, "ypr"); //Specify the publisher name
 	 nh.param<std::string> ("subscriber_topic", subscriber_topic, "/fmBSP/S0_rx_msg"); //Specify the subscriber name
	 nh.param<int> ("update_frequency", update_frequency, 50); //Specify the subscriber name



	ros::Subscriber serialSub = n.subscribe(subscriber_topic, 1, yprCallback);
	ros::Publisher	yprPub = n.advertise<fmMsgs::ypr>(publisher_topic, 1);

	ros::Rate loop_rate(update_frequency);

	while (ros::ok())
	{
		yprPub.publish(ypr);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
