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

//! Get the Destination if the id is known, otherwise NULL
Destination Road::find_dest(int id) const
{
  if(id == _start.id())
    return _start;
  if(id == _end.id())
    return _end;
  return NULL;
}

/*!
 * Method called after a changment of any
 * Destination attribute
 */
void Road::update_road()
{
  _angle = Calcul::angle(_start, _end);
  _distance = Calcul::distance(_start, _end);
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
