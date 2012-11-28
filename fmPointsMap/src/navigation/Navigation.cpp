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
#define X_HOME 400
#define Y_HOME 1200

using namespace std;

//-----------------------------------------------------------------------
// Accessors
//-----------------------------------------------------------------------
/*!
 * Return a default point (go Home) for now.
 */
Destination Navigation::goal() const
{
  Destinations home;
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
void Navigation::update_angle() {
  Point current_position;
  current_position.set(2000,500);
}

/*!
  * Calculate the angle between two points.
  * If 1st parameter is NULL, it takes the current position
  */
static double Navigation::angle(Point p1, Point p2) {
  return 0.0;
}

/*!
 * Calculate the angle between two points.
 * If 1st parameter is NULL, it takes the current position
*/
static int Navigation::distance(Point p1,Point p2) {
 return 0; 
}