//=======================================================================
// Basic C++: class Navigation
//      Specifications of class Navigation
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
 * \file Navigation.h
 *
 * \brief Definition of class Navigation.
 */

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_
#include "Point.h"
#include "Map.h"

using namespace std;

/*!
 * \brief Class Navigation.
 *
 * The Navigation class has the Map of the area,
 * know the roads, initiate the robot (angle)... *MAKE A BETTER DESCRIPTION*
 */
class Navigation
{
private:
  Map _map; //!< the map with the points, and the roads
  double _current_angle; //!< current angle (in degree, not radian, according to the trigonometric's direction)
  int _destination; //!< current destination. We use the ID of the destination from the Map
  
public:
  // Constructors
  //! Regular constructor.
  Navigation(Map map, int current_angle, int destination)
    : _map(map), _current_angle(current_angle), _destination(destination) { }

  // Accessors
  //! Get the map
  Map map() const {return _map;}
  //! Get the Destinator's faced direction in degree
  double orientation() const {return _current_angle;}
  //! Get the current destination
  Point goal() const;

  // Mutators
  //! Set a new map.
  void set_map(Map map) {_map = map;}
  //! Set a new destination. If destination not reachable from the current position find the shortest path.
  void set_destinations(int destinations) {_destination = destinations;}

  //!Initiate the Navigation class, calculting the angle...
  void initiation();
  //!Update the current angle, will make the robot move!
  void update_angle();

   //! Check if the distance between current position and destination is fair enough.
  bool is_area_reached();

  
  // Static methods
  //! Calculate the angle.
  static double angle(Point p1, Point p2);  
  //! calculate the distance between the points paramaters.
  static int distance(Point p0, Point p1);
};

#endif