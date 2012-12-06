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

#define _SMALL_DIST_ 50

#include "Map.h"
using namespace std;

/*!
  * If Destinatior is within a destination's area
  * the destination's id is returned else -1 
  */
int Map::area(Point current_position)
{
  for (list<Destination>::iterator it=_destinations.begin();
       it!=_destinations.end(); ++it)
  {
    if(Calcul::distance(current_position, (*it)) < _SMALL_DIST_)
      return (*it).id();
  }
  return -1;
}