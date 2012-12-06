//=======================================================================
// Basic C++: class Navigation
// Specifications of class Navigation
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
* \file Navigation.cpp
*
* \brief Implementation of class Navigation.
*/

#include "Navigation.h"

using namespace std;

//-----------------------------------------------------------------------
// Functions for the navigation itself
//-----------------------------------------------------------------------

Navigation::Navigation(ros::NodeHandle nh, Map map)
: _map(map), _current_position(Point()), _current_angle(-1),
  _destination(-1), _update_angle(true), _listening(false)
{
  ROS_INFO("Navigation start");
  _motor_power_pub = nh.advertise<fmMsgs::motor_power>(_TOPIC_MOTOR_,
						        _MAX_MESSAGES_);
  speed(0.8,0.8);

  _current_destination = NULL;
  _current_destination_turning = NULL;
  _current_angle = NULL;

  ros::Rate loop_rate(_FREQUENCE_);
  while (ros::ok() && _current_angle == -1)
  {
    _motor_power_pub.publish(_motor_power_msg);
    ros::spin();
    loop_rate.sleep();
  }
  _update_angle = false;
  go_back();
  _update_angle = true;
}

//! Start the routine
void Navigation::start()
{

}

int Navigation::check_default_value()
{
  int error = 0;
  if(_current_position.is_default())
  {
    ROS_ERROR("current position,");
    error += 1;
  }
  if(_destination == -1)
  {
    ROS_ERROR("destination,");
    error += 10;
  }
  if(_current_angle == -1)
  {
    ROS_ERROR("current angle,");
    error += 100;
  }
  if(error > 0)
    ROS_ERROR(" hasn't been set!");
  return error;
}

//!< Set the speed to the msg *DOES NOT PUBLISH*
void Navigation::speed(double right, double left)
{
  _motor_power_msg.power_right = (right > -1) ? right : -1;
  _motor_power_msg.power_right = (right <  1) ? right :  1;
  _motor_power_msg.power_left  = (left  > -1) ?  left : -1;
  _motor_power_msg.power_left  = (left  >  1) ?  left :  1;
}