#include "ros/ros.h"
#include "navigation/Navigation.h"
#include "../../fmControllers/src/motor_controller/motor_controller.h"
#include "fmMsgs/motor_power.h"
#include "fmMsgs/gtps.h"
#define _FREQUENCE_ 500
#define _MAX_MESSAGES_ 1

void printPosition(const fmMsgs::gtpsConstPtr& msg)
{
  cout << "x: " << msg->x  << ", y: " << msg->y << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_map");
  ros::NodeHandle nh;

  /* Declaration of the topic to publish */
  ros::Publisher motor_power_pub = nh.advertise<fmMsgs::motor_power>("/fmControllers/motor_power", _MAX_MESSAGES_);
  fmMsgs::motor_power motor_power_msg;
  /* Declaration of the topic to publish */
  ros::Subscriber sub = nh.subscribe("fmMsgs/gtps_position/10522", _MAX_MESSAGES_, printPosition);

  /* Motor msg filling (without header): */
  motor_power_msg.power_right = 0.2;
  motor_power_msg.power_left = 0.2;
/*
  ros::Rate loop_rate(_FREQUENCE_);

  while (ros::ok())
  {
    motor_power_pub.publish(motor_power_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
*/
  ros::spin();
  return 0;
}