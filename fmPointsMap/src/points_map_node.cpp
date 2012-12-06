#include "ros/ros.h"
#include "navigation/Navigation.h"

#define _MOTOR_TOPIC_ "/fmControllers/motor_power"
#define _GTPS_TOPIC_  "/fmSensors/gtps_position/10522"
#define _MAX_MESSAGES_ 1

int main(int argc, char **argv)
{
  //double first_angle;
  Point first_position;
  ros::init(argc, argv, "point_map");
  ros::NodeHandle nh;
  
  /** test **/
  Map map;
  /** test **/
  
  /* Declaration of the navigation system */
  Navigation nav(nh, map);
  ros::Subscriber sub = nh.subscribe(_GTPS_TOPIC_, _MAX_MESSAGES_, &Navigation::update, &nav);
  nav.start();

  return 0;
}