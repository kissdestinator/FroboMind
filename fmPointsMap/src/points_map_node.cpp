#include "ros/ros.h"
#include "navigation/Navigation.h"
#include "navigation/Destination.h"
#include "../../fmControllers/src/motor_controller/motor_controller.h"
#include "fmMsgs/motor_power.h"
#include "fmMsgs/gtps.h"
#include "navigation/Point.h"

#define _FREQUENCE_ 500
#define _MAX_MESSAGES_ 1
#define _TOPIC_MOTOR_ "/fmControllers/motor_power"
#define _MAX_MESSAGES_ 1

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_map");
  ros::NodeHandle nh;
  /* Declaration of the topic to publish */
  ros::Publisher motor_power_pub = nh.advertise<fmMsgs::motor_power>(_TOPIC_MOTOR_, _MAX_MESSAGES_);
  fmMsgs::motor_power motor_power_msg;
  /* Declaration of the service to get the position */
  Navigation nav;
  nav.set_client(nh);
  nav.update_position();  
//current position - this would come from the service
 
  /* Motor msg filling : */
  motor_power_msg.power_right = + 0.2;
  motor_power_msg.power_left = + 0.2;
  Point new_position;
  if ((new_position.x() +50 >= nav.position().x())
    || (new_position.y() +50 >= nav.position().y()))
  {
   motor_power_msg.power_left = 0;
   motor_power_msg.power_right = 0;
  }
  nav.update_angle();
  nav.set_client(nh);
  nav.update_position();  

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