//=======================================================================
// Basic C++: class Calcul
//      Specification of class Calcul
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
 * \file Calcul.h
 *
 * \brief Definition of class CALCUL.
 */

#ifndef _CALCUL_H_
#define _CALCUL_H_

#include <math.h>
#include "Destination.h"

#define _CIRCLE_CIRCUM_ 1256.637
#define _RADIUS_ 200

using namespace std;

/*!
 * \brief Class Calcul.
 *
 * The Calcul class makes all
 * calculations needed for navigation
 */
class Calcul
{
public:
/*!
 * Calculate the angle between the two destinations attribute
 */
  static double distance(Point p1, Point p2){
    int disx = p2.x() - p1.x();
    int disy = p2.y() - p1.y();
    return (sqrt(pow(disx,2) + pow(disy,2)));
  }

/*!
 * Calculate the distance between the two destinations param
 */
  static double angle(Point p1, Point p2){
    int difx = p2.x()-p1.x();
    int dify = p2.y()-p1.y();
    double angleDeg = atan2(dify, difx) * 180 / M_PI;
    return (angleDeg >= 0) ? angleDeg : (angleDeg+360);
  }

/*!
 * Calculate the distance between two destinations
 * using a circle!
 * Assuming that one of the wheels will stay
 * at the same spot and the GT-PS is in the middle
 * of the robot. We're also asuming just for now that 
 * we turn only clockwise.
 * NEW: If turning Destination ID is even turn clockwise,
 * if odd - anticlockwise.
 */
  static double distance_circle(double current_angle, Point current_position, Destination d1)
  {
    double dist = 0.0;
    double angle_to_dest = angle(current_position, d1);
      if (current_angle > angle_to_dest)
      {
       dist = ((current_angle - angle_to_dest)/180) * _CIRCLE_CIRCUM_; 
      }
      else if (current_angle < angle_to_dest)
      {
        dist = (((current_angle - angle_to_dest)+360)/180) * _CIRCLE_CIRCUM_;
      }
      else // (current_angle == angle_to_dest)
      {
        dist = 0.0;
      }
    return (d1.id() % 2) ? (_CIRCLE_CIRCUM_ - dist) : dist;
  }

  /*!
   * Calculate the destination to reach to make the
   * robot turn and face the good direction to the point
   * id's destination is even if turn right faster
   * 			  odd if turn left faster
   */
  static Destination turning_aim(double current_angle, Point current_position, Destination d1)
  {
    double angle_to_dest = angle(current_position, d1);          
    Destination d2; 
    d2.add_destination(d1.id());
    d2.set(current_position.x() + _RADIUS_ * cos(angle_to_dest * M_PI / 180), current_position.y() + _RADIUS_ * sin(angle_to_dest * M_PI / 180));
    double angle_diff = current_angle - angle_to_dest;
    if (fabs(angle_diff) >= 180)
    {
      (angle_diff > 0) ? d2.set_id(-3) : d2.set_id(-2);
    }
    else
    {
      (angle_diff >= 0) ? d2.set_id(-2) : d2.set_id(-3);
    }
    return d2;
  }
};

#endif
