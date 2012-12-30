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
#include "ros/ros.h"
#include "Destination.h"
#include "Calcul.h"

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
  list<Destination> _destinations; //!< *All* the roads from the csv file

public:
  // Constructors
  //! Regular constructor.
  Map(list<Destination> roads = list<Destination>())
    : _destinations(roads) {
      //Next coordinates had been choosen as destinations point:
      _destinations.push_front(Destination(0,  141,  643));
      _destinations.push_front(Destination(1, 2255,  660));
      _destinations.push_front(Destination(2, 3480, 1730));
      _destinations.push_front(Destination(3, 3815,  778));
    }

  /*!
   * Return the Destination according the id sent
   * If not on the list return a destination with id : -1
   */
  Destination destination(int id);
  /*!
   * If Destinatior is within a destination's area
   * the destination's id is returned else -1 
   */
  int area(Point current_position);
};
#endif