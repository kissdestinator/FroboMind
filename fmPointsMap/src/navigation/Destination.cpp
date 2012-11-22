//=======================================================================
// Basic C++: classe Destination
//      Specification of class Destination
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
 * \file Destination.cpp
 *
 * \brief Implementation of class Destination.
 */

#include "Destination.h"
using namespace std;


//-----------------------------------------------------------------------
// Mutators
//-----------------------------------------------------------------------

//! Add a new destination if this one does not exist
void add_destination(int p)
{
  _destinations.push_back(p);
  
}

//! Remove a destination
void rm_destination(int p)
{
  _destinations.remove(p);
  
}
//-----------------------------------------------------------------------
// Relational operators (implemented as friend functions)
//-----------------------------------------------------------------------

/*!
 * All members must be equal except.
 *
 * \note equality of the destinations NOT checked.
 */
bool operator==(Destination p1, Destination p2)
{
    return p1._x == p2._x
        && p1._y == p2._y
        && p1._id == p2._id;
}

//-----------------------------------------------------------------------
// IO operators
//-----------------------------------------------------------------------

/*!
 * We produce a printable form, with the name of the month.
 *
 * \param[in,out] os the output stream
 * \param[in] d the destination to print
 */
ostream& operator<<(ostream& os, Destination d)
{
  os << '#' << d._id << (Point)d << "; destinations: ";
  list<int>::iterator i = d._destinations.begin();
  os << "#" << *(i++);
  for(; i != d._destinations.end(); ++i)
    os << ", #" << *i;
  os << '.';
  return os;
}