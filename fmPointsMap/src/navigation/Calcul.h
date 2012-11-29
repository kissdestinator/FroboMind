
//=======================================================================
// Basic C++: classe Calcul
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
#include "Point.h"



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
 * Calculate the distance between the two destinations attribute
 * from the lower id of any destination to the higher
 * (the same real road has two different angle...)
 */
  static double angle(Point p1, Point p2){
    int difx = p2.x()-p1.x();
    int dify = p2.y()-p1.y();
    double angleDeg = atan2(dify, difx) * 180 / M_PI;
    return (angleDeg >= 0) ? angleDeg : (angleDeg+360);
  }
};

#endif
