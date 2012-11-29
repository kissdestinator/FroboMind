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
  update_angle();
}

/*!
* Update the angle value.
* *This function will make the robot move*
*/
void Navigation::update_angle()
{
  if(moved())
  {
    _current_angle = Calcul::angle(_old_position, _current_position);
    _old_position = _current_position;
  }
}
//!Update the current position
void Navigation::update_position(int x, int y)
{
  _current_position.set(msg->x, msg->y);
}

bool Navigation::moved() const
{
  return (_SMALL_DIST_ >= Calcul::distance(_old_position, _current_position));
}

/*!
 * Update the position and the angle if the robot moved enough
 */
void Navigation::update(const fmMsgs::gtps::ConstPtr& msg)
{
  update_position(msg->x, msg->y);
  update_angle();
}

/*!
  * Calculate the angle between two points.
  * If 1st parameter is NULL, it takes the current position
  */
//double Navigation::angle(Point p1, Point p2) {
  //return 0.0;
//}

/*!
 * Calculate the angle between two points.
 * If 1st parameter is NULL, it takes the current position
*/
int Navigation::distance_to_destination()
{
  return int(Calcul::distance(_current_position,
			     _map.find_destination(_destination)));
}