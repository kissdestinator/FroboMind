#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "fmMsgs/serial.h"
#include "fmMsgs/desired_speed.h"

 	 ros::NodeHandle n;
 	 ros::Publisher pololu_pub = n.advertise<fmMsgs::serial>("pololu", 1000);

	void target_handler(const fmMsgs::desired_speedConstPtr & msg){
		int desired_speed_left = (msg->speed_left +1)*127;
		int desired_speed_right = (msg->speed_right +1)*127;
		char pololu_data[6] = {255,0,desired_speed_left,255,1,desired_speed_right};
	    std::string data;
	    data.assign(pololu_data,6);
	    fmMsgs::serial msg2;
	    msg2.data = data;

	  pololu_pub.publish(msg2);
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pololu");

  ros::Subscriber pololu_sub = n.subscribe<fmMsgs::desired_speed>("speed_from_joystick",1,target_handler);

  ros::spin();

  return 0;
}
