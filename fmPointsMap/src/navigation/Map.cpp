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
Destination Map::find_destination(int id)
{
  list<Road>::iterator it;
  for (it=_roads.begin() ; it != _roads.end(); it++ )
    if((*it).find_dest(id) != NULL)
      return (*it).find_dest(id);
  return NULL;
}