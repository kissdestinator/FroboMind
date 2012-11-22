//=======================================================================
// Basic C++: classe Point
//      Specification of class Point
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
 * \file Point.h
 *
 * \brief Definition of class Point.
 */

#ifndef _POINT_H_
#define _POINT_H_

#include <iostream>
#include <list>

using namespace std;

/*!
 * \brief Class Point.
 *
 * Represent a point has x, y, id and list of points's id
 * reachable to allow the representation of roads.
 */
class Point
{
private:
  int _x;  //!< x coordinate
  int _y; //!< y coordinates
  int _id; //!< id of the point
  list<int> _destinations; //!< Reachable point from this one

public:
  // Constructors
  //! Regular constructor.
  Point(int x = -1, int y = -1, int id = -1)
    : _x(x), _y(y), _id(id) { }

  // Accessors
  //! Get this x coordinate.
  int x() const {return _x;}
  //! Get this y coordinates.
  int y() const {return _y;}
  //! Get this id.
  int id() const {return _id;}
  //! Get this reachable points destinations.
  list<int> destinations() const {return _destinations;}

  // Mutators
  //! Set a new x coordinate
  void setX(int x) {_x = x;}
  //! Set a new y coordinate
  void setY(int y) {_y = y;}
  //! Set a new y id
  void set_id(int id) {_id = id;}
  //! Set a new destinations list
  void set_destinations(list<int> destinations) {_destinations = destinations;}
  //! Add a new destination if this one does not exist
  void add_destination(int p);

  //! Check if the distance between the paramater and the point is low enough.
  friend bool is_area_reached(Point p);
  //! calculate the distance between the points paramaters.
  friend float distance(Point p0, Point p1);

  // Relational operators
  //! Equality operator on Point
  friend bool operator==(Point p1, Point p2);
  //! Unequality operator on Date
  friend bool operator!=(Point p1, Point p2) {return !(p1 == p2);}

  // IO operations
  //! Print out a Point.
  friend ostream& operator<<(ostream& os, Point p);
  
  void print(void);
};

#endif