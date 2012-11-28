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

//! Check if the distance between d1 position and d2 is fair enough.
bool Map::is_area_reached(Destination d1, Destination d2)
{
  cout << "Map::is_area_reached is NOT implemented!! You lazy!! Start working!!" << endl;
  return true;
}

//! Find the path from d1 to d2. *Not a prioritary function*
list<Road> Map::find_path(Destination d1, Destination d2)
{
  cout << "Map::find_path is NOT implemented!! You fucking lazy!! THINK!!" << endl;
  return list<Road>();
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