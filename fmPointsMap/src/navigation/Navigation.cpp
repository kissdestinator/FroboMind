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
  _destination(-1), _update_angle(true), _listening(false), _not_first_update(false)
{

  ROS_INFO("[Navigation::Navigation] constructing");
  _motor_power_pub = nh.advertise<fmMsgs::motor_power>(_TOPIC_MOTOR_,
						        _MAX_MESSAGES_);
}

//! Make the robot return 3 cm backward without updating the angle
void Navigation::go_back()
{
  _update_angle = false;
  ros::Rate loop_rate(_FREQUENCE_);

  _GO_BACKWARD_
  ROS_INFO("[Navigation::go_back] entering loop");
  while (ros::ok() && !moved())
  {
    _motor_power_pub.publish(_motor_power_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("[Navigation::go_back] leaving loop");

  _update_angle = true;
}

void Navigation::initialisation()
{
  ros::Rate loop_rate(_FREQUENCE_);

  // initialisation of the angle
  _GO_FORWARD_
  while(ros::ok() && _current_angle <= -1)
  {
   // ROS_INFO("[Navigation::Navigation] angle: %g", _current_angle);
    //ROS_INFO("[Navigation::Navigation] looping");
    _motor_power_pub.publish(_motor_power_msg);
    //ROS_INFO("[Navigation::Navigation] publish");
    ros::spinOnce();
    //ROS_INFO("[Navigation::Navigation] spin");
    loop_rate.sleep();
    //ROS_INFO("[Navigation::Navigation] snor");
  }
  ROS_INFO("[Navigation::initialisation] leaving loop");
  go_back();
  int destination = -2;
  if((destination = is_known_destination()) > -1)
  {
    _current_destination = _current_position;
    _destination = destination;
  }
  else
  {//Go to destination 0
    _listening = false;
    _current_destination = _map.destination(0);
    ROS_INFO("[Navigation::initialisation] _current_destination:#%d(%d, %d)",
	  _current_destination.id(),
	  _current_destination.x(),
	  _current_destination.y());
    go();
  }
  //Check if we are at a known destination.
  //else go to 0-id-ed destination.
}
 //!< Check if Desinator is at known destination
int Navigation::is_known_destination()
{
  int destination_found = _map.area(_current_position);
  if ( destination_found == -1)
    ROS_INFO("[Navigation::is_known_destination] I'm lost!");
  else
    ROS_INFO("[Navigation::is_known_destination] destination found %d",destination_found);
  return destination_found;
}

int Navigation::check_default_value()
{
  int error = 0;
  if(_current_position.is_default())
  {
    ROS_ERROR("current position,");
    error -= 1;
  }
  if(_current_angle == -1)
  {
    ROS_ERROR("current angle ");
    error -= 10;
  }
  if(error > 0)
    ROS_ERROR(" hasn't been set!");
  return error;
}

//!< Set the speed to the msg *DOES NOT PUBLISH*
void Navigation::speed(double right, double left)
{
  right = (right > -1) ? right : -1;
  right = (right <  1) ? right :  1;
  left  = (left  > -1) ?  left : -1;
  left  = (left  <  1) ?  left :  1;

  ROS_INFO("[Navigation::speed] speed set to: right=%g, left=%g",
	   right, left);
  _motor_power_msg.power_right = right;
  _motor_power_msg.power_left  = left;
}

//! Start the routine
void Navigation::start()
{
  initialisation();
  int error = -1;
  if((error = check_default_value()) _IS_NOT_GOOD_)
  {
    ROS_ERROR("[Navigation::start] Unusable value:%d", error);
    ros::shutdown();
  }

  //Everything is ok, we can start the routine:
  ROS_INFO("[Navigation::start] entering in the main loop");
  while (ros::ok())
  {
    _listening = true;
    //if Destinator is *NOT* in the destination's area set
    if(_destination != _map.area(_current_position))
      go();
  }
}