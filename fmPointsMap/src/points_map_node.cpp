#include "ros/ros.h"
#include "navigation/Navigation.h"
#include "navigation/Destination.h"
#include "navigation/Calcul.h"
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
  //double first_angle;
  Point first_position;
  ros::init(argc, argv, "points_map");
  ros::NodeHandle nh;
  /* Declaration of the topic to publish */
  ros::Publisher motor_power_pub = nh.advertise<fmMsgs::motor_power>(_TOPIC_MOTOR_, _MAX_MESSAGES_);
  fmMsgs::motor_power motor_power_msg;
  /* Declaration of the service to get the position */
  Navigation nav;
  nav.set_client(nh);
  nav.update_position();  
  first_position = nav.position();
  /* Motor msg filling : */
  motor_power_msg.power_right = + 0.2;
  motor_power_msg.power_left = + 0.2;
  if ((nav.position().x() >= first_position.x() +50 )
    || (nav.position().y() >= first_position.y()+50))
  {
   motor_power_msg.power_left = 0;
   motor_power_msg.power_right = 0;
   nav.update_angle();
   // to set the first angle using Calcul.h 
   //that we made or the function from Road.h
   //first_angle = Calcul::angle(nav.position(), first_position);
   //nav.set_orientation(first_angle);
   motor_power_msg.power_right = 0.2;
   motor_power_msg.power_left = 0.2;
  }
   if ((nav.position().x() >= first_position.x())
    || (nav.position().y() >= first_position.y()))
    {
       motor_power_msg.power_left = 0;
       motor_power_msg.power_right = 0;
    }
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
