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
	ros::init(argc, argv, "imu_ypr_extractor");

	ros::NodeHandle n;

	ros::Subscriber serialSub = n.subscribe("/fmBSP/S0_rx_msg", 1, yprCallback);
	ros::Publisher	yprPub = n.advertise<fmMsgs::ypr>("ypr", 1);

	while (ros::ok())
	{
		yprPub.publish(ypr);

		ros::spinOnce();
	}

	return 0;
}
