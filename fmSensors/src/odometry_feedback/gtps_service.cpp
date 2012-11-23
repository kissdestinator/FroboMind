#include "ros/ros.h"
#include "fmMsgs/gtps.h"
#include "fmSensors/GTPS.h"
#include "GTPS_response.h"


#define _MAX_MESSAGES_ 1
#define _GTPS_TOPIC_ "fmMsgs/gtps_position/10522"

class Listener
{
public:
  void callback(const fmMsgs::gtps::ConstPtr& msg)
  {}
};

int main(int argc, char **argv)
{
  /* initialize ros usage */
  ros::init(argc, argv, "gtps_service");
  ros::NodeHandle nh;
  
  GTPS_response response = GTPS_response();
  Listener listener;
  ros::Subscriber sub = nh.subscribe(_GTPS_TOPIC_, _MAX_MESSAGES_, &GTPS_response::update, &response);  
  /**
  GTPS_response response();
  
   * Each time a message is published in the gtps topic the
   * GTPS_response::update function is called
   
  ros::Subscriber sub = nh.subscribe(_GTPS_TOPIC_, _MAX_MESSAGES_,
		&GTPS_response::update, &response);
  */
  ros::spin();
  return 0;
}