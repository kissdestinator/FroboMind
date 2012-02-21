#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "fmMsgs/serial.h"
#include "fmMsgs/desired_speed.h"
#include "pololu_motor_controller.h"

int main(int argc, char **argv)
{
	/* initialize ros usage */
	ros::init(argc, argv, "pololu_motor_controller");

	/* parameters */
	std::string serial_pub_topic;
	std::string desired_speed_sub_topic;

	/* private nodehandlers */
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	/* read parameters from ros parameter server if available otherwise use default values */
	n.param<std::string> ("serial_publisher_topic", serial_pub_topic, "pololu_serial"); //Specify the publisher name
	n.param<std::string> ("desired_speed_subscriber_topic", desired_speed_sub_topic, "desired_speed"); //Specify the publisher name

	PololuMotorController pmc;

	pmc.pololu_pub = n.advertise<fmMsgs::serial>(serial_pub_topic.c_str(), 1);
	pmc.pololu_sub = n.subscribe<fmMsgs::desired_speed>(desired_speed_sub_topic.c_str(),1,&PololuMotorController::callbackHandler,&pmc);

	ros::spin();

	return 0;
}
