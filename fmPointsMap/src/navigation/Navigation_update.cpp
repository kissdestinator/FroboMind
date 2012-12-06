//=======================================================================
// Basic C++: class Navigation
// Specifications of class Navigation
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
* \file Navigation_update.cpp
*
* \brief Implementation of class Navigation.
*/

#include "Navigation.h"

using namespace std;

/*!
* Update the angle value.
* *This function will make the robot move*
*/
void Navigation::update_angle()
{
  if(moved() && _update_angle)
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