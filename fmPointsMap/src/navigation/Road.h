//=======================================================================
// Basic C++: classe Road
//      Specification of class Road
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou - Grzegorczyk Lucas
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
 * \file Road.h
 *
 * \brief Definition of class Road.
 */

#ifndef _ROAD_H_
#define _ROAD_H_
#define DEFAULT_ANGLE -1
#define DEFAULT_DIST -1
#define DEFAULT_VALUE (_angle == -1 || _distance == -1)

#include <iostream>
#include <list>
#include "Destination.h"
#include "Point.h"

using namespace std;

/*!
 * \brief Class Road.
 *
 * Represent road between two destinations
 * calculate the angle, the distance of the road.
 */
class Road
{
private:
  Destination _start;  //!< Beginning of road
  Destination _end; //!< End of the road
  int _id; //!< id of the road *nothing related with the destination's id*
  double _angle; //!< Angle of the road
  int _distance; //!< Distance between the start and the end of the road

  //! Update the angle and the distance 
  void update_road();
  //! Calculate and set the angle using class' attribute from the lower id of any destination to the higher (the same real road has two different angle...)
  double calcul_angle();
  //! Calculate and set the distance using class' attribute
  int calcul_distance();

public:
  // Constructors
  //! Regular constructor.
  Road(Destination start, Destination end, int id = -1, double angle = DEFAULT_ANGLE, int distance = DEFAULT_DIST)
    : _start(start), _end(end), _id(id), _angle(angle), _distance(distance) { if(DEFAULT_VALUE) update_road(); }

  // Accessors
  //! Get the start destination
  Destination start() const {return _start;}
  //! Get the end destination
  Destination end() const {return _end;}
  //! Get the id of the road
  int id() const {return _id;}
  //! Get the angle of the road
  int angle() const {return _angle;}
  //! Get the distance of the road
  int distance() const {return _distance;}

  // Mutators
  //! Set a new start and update the road
  void set_start(Destination start) {_start = start;update_road();}
  //! Set a new end and update the road
  void set_end(Destination end) {_end = end;update_road();}
  //! Set a new y id
  void set_id(int id) {_id = id;}

  // Relational operators
  //! Equality operator on Road
  friend bool operator==(Road r1, Road r2);
  //! Unequality operator on Road
  friend bool operator!=(Road r1, Road r2) {return !(r1 == r2);}

  // IO operations
  //! Print out a Point.
  friend ostream& operator<<(ostream& os, Road r);
};

#endif