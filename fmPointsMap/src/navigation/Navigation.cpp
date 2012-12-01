//=======================================================================
// Basic C++: class Navigation
//      Specifications of class Navigation
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
#include "Calcul.h"

#define X_HOME 400
#define Y_HOME 1200
//each time the robot move to 30 mm the angle is updated
#define _SMALL_DIST_ 30
#define _NO_UPDATE_ _update==false
#define _TURNING_ _turning==true
#define _NO_BACKWARD_ _update==false
#define _FREQUENCE_ 500


using namespace std;

//-----------------------------------------------------------------------
// Accessors
//-----------------------------------------------------------------------
/*!
 * Return a default point (go Home) for now.
 */
Destination Navigation::goal() const
{
  Destination home;
  home.set(X_HOME, Y_HOME);
  return home;
  //update the description of the function
  //return _map.getPoint(_destination);
}

//-----------------------------------------------------------------------
// Functions for the navigation itself
//-----------------------------------------------------------------------

/*!
 * Make the initialisation with the angle of the robot and so on
 * Do not make it in the constructor to allow the user to choose
 * the initialisation date
 * *This function will make the robot move*
 */
void Navigation::initiation() {
  speed(0.2);

  ros::Rate loop_rate(_FREQUENCE_);
  while (ros::ok() && _current_angle == -1)
  {
    _motor_power_pub.publish(_motor_power_msg);
    ros::spin();
    loop_rate.sleep();
  }
  _update = false;
  go_back();
  _update = true;
}

//!< Set the speed to the msg *DOES NOT PUBLISH*
void Navigation::speed(double speed)
{
  _motor_power_msg.power_right = (speed > -1) ? speed : -1;
  _motor_power_msg.power_right = (speed <  1) ? speed :  1;
  _motor_power_msg.power_left  = (speed > -1) ? speed : -1;
  _motor_power_msg.power_left  = (speed >  1) ? speed :  1;
}

//! Make the robot return 3 cm backward without updating the angle
void Navigation::go_back()
{
  speed(-0.2);

  ros::Rate loop_rate(_FREQUENCE_);
  while (ros::ok() || !moved())
  {
    _motor_power_pub.publish(_motor_power_msg);
    ros::spin();
    loop_rate.sleep();
  }
}

/*!
* Update the angle value.
* *This function will make the robot move*
*/
void Navigation::update_angle()
{
  if(moved() && _NO_UPDATE_ && !_TURNING_)
  {
    _current_angle = Calcul::angle(_old_position, _current_position);
    _old_position = _current_position;
    ROS_INFO("current angle updated:%g", _current_angle);
  }
}

//!Update the current position
void Navigation::update_position(int x, int y)
{
  _current_position.set(x, y);
}

bool Navigation::moved() const
{
  return (_SMALL_DIST_ >= Calcul::distance(_old_position,
					    _current_position));
}

/*!
 * Update the position and the angle if the robot moved enough
 */
void Navigation::update(const fmMsgs::gtps::ConstPtr& msg)
{
  update_position(msg->x, msg->y);
  update_angle();
}

//! Method called at each message publish from web node.

void Navigation::set_new_destination(const fmMsgs:: web:: ConstPtr& msg)
{
  Destination new_dest = _map.find_destination(msg->id);
 if(new_dest == NULL)
 {
  _current_destination = Destination(_current_position);
 }
 else
 {
   _current_destination = new_dest;
 }
}

/*!
 * Calculate the angle between two points.
 * If 1st parameter is NULL, it takes the current position
 */
int Navigation::distance_to_destination()
{
  if(!_TURNING_)
    return (_current_destination == NULL) ?
      int(Calcul::distance(_current_position,
			   _current_destination))
      : 0 ;//if no destination return 0
  else
    return int(Calcul::distance_circle(_current_position,
				       _current_destination_turning));
}