//=======================================================================
// Basic C++: class Navigation
// Specifications of class Navigation
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

#include "../../../fmControllers/src/motor_controller/motor_controller.h"
#include "fmMsgs/motor_power.h"
#include "fmMsgs/gtps.h" //Message from the topic
#include "fmMsgs/web.h" //Message from the web topic
#include "Calcul.h"
#include "Map.h"
#include "ros/ros.h"

#define _TOPIC_MOTOR_ "/fmControllers/motor_power"
#define _MAX_MESSAGES_ 1
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
  Destination _current_destination;//!< Current Destinator's aim
  Destination _current_destination_turning;//!< Current Destinator's aim while turning
  Point _old_position;//!< "old" Destinator's known position: needed to calculate the angle
  volatile double _current_angle; //!< current angle (in degree, not radian, according to the trigonometric's direction)
  /**
   * The angle need to be updated as much as possible. However if it's done within a
   * too short time the value could be useless.
   * Then we use the "Point _old_position" to update with the _current_position if
   * the old one is old enough! (using either the time stamp sent by the GTPS service)
   */
  int _destination; //!< current destination. We use the ID of the destination from the Map
  bool _update; //!< if false do not update the angle
  bool _turning; //!< if turning, no update angle
  bool _listening; //! false before the end of the initialisation
  ros::Publisher _motor_power_pub; //!< Publisher to 
  fmMsgs::motor_power _motor_power_msg;  //!< Msg to publish to motor power topic

  //!< Return true if the moved enough to update the angle
  bool moved() const;
  /*!
   * Return greater than 0 if all the
   * default attribute's value has been changed
   */
  int check_default_value();
  //!< Set the speed to the msg *DOES NOT PUBLISH*
  void speed(double right, double left);
  //!< Make the robot return 3cm back after init
  void go_back();

  //! Update the current angle
  void update_angle();
  //! Update the current position
  void update_position(int x, int y);
  //! Initiate the Navigation class, calculting the angle...
  void initialisation();
  //! Correct the angle
  void face_destination();
  //! Go forward to the destination
  void go();
  //! Make the robot reach the destination's area
  void move_to_destination();
  

public:
  // Constructors
  //! Regular constructor.
  Navigation(ros::NodeHandle nh,
	      Map map = Map(),
	      Point position = Point(),
	      int destination = -1)
    : _map(map), _current_position(position), _current_angle(-1),
      _destination(destination), _update(true), _turning(false)
    {_motor_power_pub = nh.advertise<fmMsgs::motor_power>(_TOPIC_MOTOR_, _MAX_MESSAGES_); _listening = false;}

  // Accessors
  //! Get the map
  Map map() const {return _map;}
  //! Get the Destinator's faced direction in degree
  double orientation() {return _current_angle;}
  //! Get the Destinator's current position
  Point position() {return _current_position;}
  //! Get the current destination
  Destination goal() const;

  // Mutators
  //! Set a new map.
  void set_map(Map map) {_map = map;}
  //! Set a new destination. If destination not reachable from the current position find the shortest path.
  void set_destinations(int destinations) {_destination = destinations;}
  //! Start the routine
  void start();
  //! Method called at each message publish from gtps topic.
  void update(const fmMsgs::gtps::ConstPtr& msg);
  //! Method called at each message publish from web node.
  void set_new_destination(const fmMsgs:: web:: ConstPtr& msg);
  //! Check if the distance between current position and destination is fair enough.
  bool is_area_reached();
  //! calculate the distance between the points paramaters.
  int distance_to_destination();
};

#endif