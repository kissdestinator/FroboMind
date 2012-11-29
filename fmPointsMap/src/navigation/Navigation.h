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

#include "fmMsgs/gtps.h"	//Message from the topic
#include "Calcul.h"
#include "Map.h"
#include "ros/ros.h"

#define _GTPS_SRV_ "gtps_service"

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
  Map _map; //!< map with the points, and the roads
  Point _current_position;//!< last Destinator's known position
  Point _old_position;//!< "old" Destinator's known position: needed to calculate the angle
  double _current_angle; //!< current angle (in degree, not radian, according to the trigonometric's direction)
  /**
   * The angle need to be updated as much as possible. However if it's done within a
   * too short time the value could be useless.
   * Then we use the "Point _old_position" to update with the _current_position if
   * the old one is old enough! (using either the time stamp sent by the GTPS service)
   */
  int _destination; //!< current destination. We use the ID of the destination from the Map
  //!< Return true if the moved enough to update the angle
  bool moved() const;

  //!Update the current angle
  void update_angle();
  //!Update the current position
  void update_position(int x, int y);
public:
  // Constructors
  //! Regular constructor.
  Navigation(Map map = Map(), Point position = Point(), int current_angle = -1, int destination = 0)
    : _map(map), _current_position(position), _current_angle(current_angle), _destination(destination)
    { }

  // Accessors
  //! Get the map
  Map map() const {return _map;}
  //! Get the Destinator's faced direction in degree
  double orientation() const {return _current_angle;}
  //! Get the Destinator's current position
  Point position() const {return _current_position;}
  //! Get the current destination
  Destination goal() const;

  // Mutators
  //! Set a new map.
  void set_map(Map map) {_map = map;}
  //! Set a new destination. If destination not reachable from the current position find the shortest path.
  void set_destinations(int destinations) {_destination = destinations;}
  //! Set the first angle, the orientation of the robot.
  //void set_orientation(double angle){_current_angle = angle;}
  //!Initiate the Navigation class, calculting the angle...
  void initiation();
  //! Method called at each message publish from gtps topic.
  void update(const fmMsgs::gtps::ConstPtr& msg);
  //! Check if the distance between current position and destination is fair enough.
  bool is_area_reached();
  //! calculate the distance between the points paramaters.
  int distance_to_destination();
};

#endif