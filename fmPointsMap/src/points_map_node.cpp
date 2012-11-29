#include "ros/ros.h"
#include "navigation/Navigation.h"
#include "navigation/Destination.h"
#include "navigation/Calcul.h"
#include "../../fmControllers/src/motor_controller/motor_controller.h"
#include "fmMsgs/motor_power.h"
#include "fmMsgs/gtps.h"
#include "navigation/Point.h"

#define _MOTOR_TOPIC_ "/fmControllers/motor_power"
#define _GTPS_TOPIC_  "/fmSensors/gtps_position/10522"
#define _MAX_MESSAGES_ 1

int main(int argc, char **argv)
{
  //double first_angle;
  Point first_position;
  ros::init(argc, argv, "points_map");
  ros::NodeHandle nh;
  /* Declaration of the navigation system */
  Navigation nav;
  ros::Subscriber sub = nh.subscribe(_GTPS_TOPIC_, _MAX_MESSAGES_, &Navigation::update, &nav);

  return 0;
}