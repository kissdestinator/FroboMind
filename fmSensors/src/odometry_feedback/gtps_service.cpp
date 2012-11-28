#include "ros/ros.h"
#include "fmMsgs/gtps.h"	//Message from the topic
#include "fmSensors/GTPS.h"	//Service to implement
#include "GTPS_response.h"	//Response class which implements

#define _MAX_MESSAGES_ 1
#define _GTPS_TOPIC_ "fmMsgs/gtps_position/10522"
#define _GTPS_SRV_ "gtps_service"

using namespace std;

void doit(const fmMsgs::gtps::ConstPtr& msg)
{
ROS_INFO("GTPS_response.h says toto!");
}

int main(int argc, char **argv)
{
  /* initialize ros usage */
  ros::init(argc, argv, "gtps_service");
  ros::NodeHandle nh;
ROS_INFO("GTPS_response.h says Hi!");
  GTPS_response service;
  ros::Subscriber sub = nh.subscribe(_GTPS_TOPIC_, _MAX_MESSAGES_, doit);//&GTPS_response::update, &service);
  ros::ServiceServer ss = nh.advertiseService(_GTPS_SRV_, &GTPS_response::response, &service);

  ros::spin();
  return 0;
}
