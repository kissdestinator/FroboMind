//=======================================================================
// Basic C++: classe Map
//      Specification of class Map
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
 * \file Map.cpp
 *
 * \brief Implementation of class Map.
 */

#include "Map.h"
using namespace std;

//! Return the Destination of the _id send as param
Destination Map::find_destination(int id) const
{
  list<Road>::iterator it;
  for (it=mylist.begin() ; it != mylist.end(); it++ )
    if((*it).find_destination(id) != NULL)
      return (*it).find_destination(id);
  return NULL;
}

//-----------------------------------------------------------------------
// IO operators
//-----------------------------------------------------------------------

/*!
 * We produce a printable form of the Map
 *
 * \param[in,out] os the output stream
 * \param[in] d the destination to print
 */
ostream& operator<<(ostream& os, Destination d)
{
  os << "You didn't code the function lazy n3rd!";
  return os;
}