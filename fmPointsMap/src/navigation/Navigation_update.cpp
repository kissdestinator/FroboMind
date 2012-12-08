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

//! Method called at each message publish from web node.
void Navigation::set_new_destination(const fmMsgs::web:: ConstPtr& msg)
{
  if(_listening) {
  Destination new_dest = _map.destination(msg->id);
  //If the destination doesn't exist,destination is set to current position
  if(new_dest.id() == -1)
  {
    _current_destination = Destination(_current_position);
    ROS_INFO("[Navigation::set_new_destination] id=%d is unknown", msg->id);
  }
  else
    _current_destination = new_dest;
    ROS_INFO("Current destination updated: (%d,%d).",
	     _current_destination.x(),
	     _current_destination.y());
  }
  else
    ROS_INFO("[Navigation::set_new_destination] id=%d is not listened",
	     msg->id);
}

//!< Return true if the moved enough to update the angle
bool Navigation::moved() const
{
//  ROS_INFO("[Navigation::moved] Distance: %g", Calcul::distance(_old_position,_current_position));
  return (_SMALL_DIST_ <= Calcul::distance(_old_position,
					    _current_position));
}

/*!
* Update the angle value.
* *This function will make the robot move*
*/
void Navigation::update_angle()
{
  if(moved() && _update_angle && _not_first_update)
  {
    _current_angle = Calcul::angle(_old_position, _current_position);
    _old_position = _current_position;
    ROS_INFO("[Navigation::update_angle] current angle updated:%g",
	     _current_angle);
  }
}

//!Update the current position
void Navigation::update_position(int x, int y)
{
  _current_position.set(x, y);
}

/*!
* Update the position and the angle if the robot moved enough
*/
void Navigation::update(const fmMsgs::gtps::ConstPtr& msg)
{
  //ROS_INFO("[Navigation::update] Old position: (%d, %d) New position: (%d, %d)", _old_position.x(), _old_position.y(), _current_position.x(), _current_position.y() );
  update_position(msg->x, msg->y);
  update_angle();
  _not_first_update = true;
}
