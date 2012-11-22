//=======================================================================
// Basic C++: classe Point
//      Specification of class Point
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
 * \file Point.cpp
 *
 * \brief Implementation of class Point.
 */

#include "Point.h"
using namespace std;
//-----------------------------------------------------------------------
// Relational operators (implemented as friend functions)
//-----------------------------------------------------------------------

/*!
 * All members must be equal except.
 *
 * \note We give a direct definition here because it is simple, but a
 * definition of this operator can be derived from operator<().
 */
bool operator==(Point p1, Point p2)
{
    return p1._x == p2._x
        && p1._y == p2._y
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
ostream& operator<<(ostream& os, Point p)
{
  os << '(' << p._x << ',' << p._y << ')';
  return os;
}