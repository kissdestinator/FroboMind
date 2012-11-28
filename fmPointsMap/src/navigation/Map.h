//=======================================================================
// Basic C++: classe Map
//      Specification of class Map
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
 * \file Map.h
 *
 * \brief Definition of class Map.
 */

#ifndef _MAP_H_
#define _MAP_H_

#include <list>
#include "Road.h"

using namespace std;

/*!
 * \brief Class Map.
 *
 * Represent a map with the roads.
 * This class calculate the "destination area" of each destination,
 * find the set of roads to follow from any point to any other
 */
class Map
{
private:
  list<Road> _roads; //!< *All* the roads from the csv file

public:
  // Constructors
  //! Regular constructor.
  Map(list<Road> roads = list<Road>())
    : _roads(roads) { }

  // Accessors
  //! Get the roads
  list<Road> roads() const {return _roads;}

  //! Check if the distance between d1 position and d2 is fair enough.
  bool is_area_reached(Destination d1, Destination d2);
  //! Find the path from d1 to d2. *Not a prioritary function*
  list<Road> find_path(Destination d1, Destination d2);

  // IO operations
  //! Human-readably-text-formated print out a Map.
  friend ostream& operator<<(ostream& os, Map m);
};

#endif