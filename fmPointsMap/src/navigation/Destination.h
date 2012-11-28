//=======================================================================
// Basic C++: classe Point
//      Specification of class Point
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
 * \file Destination.h
 *
 * \brief Definition of class Destination.
 */

#ifndef _DEST_H_
#define _DEST_H_

#include <iostream>
#include <list>
#include "Point.h"

using namespace std;

/*!
 * \brief Class Destination.
 *
 * Represent a derivated class from point
 * that has an id and list of destination's id
 * reachable to allow the representation of roads.
 */
class Destination: public Point
{
private:
  int _id; //!< id of the destination
  list<int> _destinations; //!< Reachable destination

public:
  // Constructors
  //! Regular constructor.
  Destination(int id = -1, list<int> destinations = list<int>())
    : _id(id), _destinations(destinations) { }

  // Accessors
  //! Get this id.
  int id() const {return _id;}
  //! Get this reachable points destinations.
  list<int> destinations() const {return _destinations;}

  // Mutators
  //! Set a new y id
  void set_id(int id) {_id = id;}
  //! Set a new destinations list
  void set_destinations(list<int> destinations) {_destinations = destinations;}
  //! Add a new destination if this one does not exist
  void add_destination(int p) { _destinations.push_back(p); _destinations.unique(); }
  //! Remove a destination
  void rm_destination(int p) { _destinations.remove(p); }

  // Relational operators
  //! Equality operator on Destination
  friend bool operator==(Destination d1, Destination d2);
  //! Unequality operator on Destination
  friend bool operator!=(Destination d1, Destination d2) {return !(d1 == d2);}

  // IO operations
  //! Print out a Destination.
  friend ostream& operator<<(ostream& os, Destination p);
};

#endif