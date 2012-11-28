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

using namespace std;

/*!
 * \brief Class Point.
 *
 * Represent a point has x, y, id and list of points's id
 * reachable to allow the representation of roads.
 */
class Point
{
protected:
  int _x;  //!< x coordinate
  int _y; //!< y coordinates

public:
  // Constructors
  //! Regular constructor.
  Point(int x = -1, int y = -1)
    : _x(x), _y(y) { }

  // Accessors
  //! Get this x coordinate.
  int x() const {return _x;}
  //! Get this y coordinates.
  int y() const {return _y;}

  // Mutators
  //! Set a new x coordinate
  void setX(int x) {_x = x;}
  //! Set a new y coordinate
  void setY(int y) {_y = y;}
  //! Set x and y coordinate
  void set(int x, int y) {_x = x; _y = y;}

  // Relational operators
  //! Equality operator on Point
  friend bool operator==(Point p1, Point p2);
  //! Unequality operator on Point
  friend bool operator!=(Point p1, Point p2) {return !(p1 == p2);}

  // IO operations
  //! Print out a Point.
  friend ostream& operator<<(ostream& os, Point p);
};

#endif