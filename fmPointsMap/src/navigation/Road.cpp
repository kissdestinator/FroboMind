//=======================================================================
// Basic C++: classe Road
//      Specification of class Road
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou - Grzegorczyk Lucas
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
 * \file Road.cpp
 *
 * \brief Implementation of class Road.
 */

#include "Road.h"
using namespace std;
//-----------------------------------------------------------------------
// Relational operators (implemented as friend functions)
//-----------------------------------------------------------------------
/*!
 * Calculate the angle between the two destinations attribute
 */
double Road::calcul_angle()
{
  cout << "Road::calcul_angle is NOT implemented!! Let's code!" << endl;
  return 361;
}

/*!
 * Calculate the distance between the two destinations attribute
 * from the lower id of any destination to the higher
 * (the same real road has two different angle...)
 */
int Road::calcul_distance()
{
  cout << "Road::calcul_distance is NOT implemented! Use your head instead of the other's one!" << endl;
  return 42;
}

/*!
 * Method called after a changment of any
 * Destination attribute
 */
void Road::update_road()
{
  _angle = calcul_angle();
  _distance = calcul_distance();
}

//-----------------------------------------------------------------------
// Relational operators (implemented as friend functions)
//-----------------------------------------------------------------------

/*!
 * All members must be equal except.
 */
bool operator==(Road r1, Road r2)
{
     return r1._start.x() == r2._start.x()
	 && r1._start.y() == r2._start.y()
	 && r1._end.x() == r2._end.x()
	 && r1._end.y() == r2._end.y();
}

//-----------------------------------------------------------------------
// IO operators
//-----------------------------------------------------------------------

/*!
 * We produce a printable form, with the name of the month.
 *
 * \param[in,out] os the output stream
 * \param[in] p the point to print
 */
ostream& operator<<(ostream& os, Road r)
{
  os << "Road #" << r._id << ", dist:" << r._distance << ", angle:" << r._angle << "; from ("
  << r._start.x() << ',' << r._start.y() << ") to ("
  << r._end.x() << ',' << r._end.y() << ')';
  return os;
}