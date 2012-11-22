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
using namespace std;

//-----------------------------------------------------------------------
// Accessors
//-----------------------------------------------------------------------
/*!
 * Return a default point (go Home) for now.
 */
Point Navigation::goal() const
{
  Point home;
  home.setX(400);
  home.setX(1200);
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
 */
void Navigation::initiation()
{
  update_angle();
}

/*!
 * Make the initialisation with the angle of the robot and so on
 * Do not make it in the constructor to allow the user to choose
 * the initialisation date
 */
void Navigation::update_angle()
{
  Point current_position;
  current_position.setX(2000);
  current_position.setY(500);

}
