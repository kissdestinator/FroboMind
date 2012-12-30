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
#define _SMALL_DIST_ 	30
#define _NO_BACKWARD_ 	_update==false
#define _IS_NOT_GOOD_ 	< 0
#define _FREQUENCE_ 	500
#define _AREA_ 		300
#define _AREA_TURNING_	50
#define _FORWARD_ 	0.6,0.6
#define _BACKWARD_ 	-0.6,-0.6
//#define _DIST_ 100


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
  double _current_angle; //!< current angle (in degree, not radian, according to the trigonometric's direction)
  
  /**
   * The angle need to be updated as much as possible. However if it's done within a
   * too short time the value could be useless.
   * Then we use the "Point _old_position" to update with the _current_position if
   * the old one is old enough! (using either the time stamp sent by the GTPS service)
   */
  int _destination; //!< current destination. We use the ID of the destination from the Map
  bool _update_angle; //!< if false do not update the angle (while turning + going backward)
  bool _listening; //! false before the end of the initialisation
  bool _not_first_update;  // skips first update of angle
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
  //!< Intit Destinator by making him move
  void initialisation();
  //!< Check if at  known destination
  int is_known_destination();

  //! Update the current angle
  void update_angle();
  //! Update the current position
  void update_position(int x, int y);
  //! Correct the angle
  void face_destination();
  //! Go forward to the destination
  void go();
  //! Make the robot reach the destination's area
  void move_to_destination();

public:
  // Constructors
  //! Regular constructor.
  Navigation(ros::NodeHandle nh, Map map);
  // Mutators
  //! Set a new destination. If destination not reachable from the current position find the shortest path.
  void set_destinations(int destinations) {_destination = destinations;}
  //! Start the routine
  void start();
  //! Method called at each message publish from gtps topic.
  void update(const fmMsgs::gtps::ConstPtr& msg);
  //! Method called at each message publish from web node.
  void set_new_destination(const fmMsgs:: web:: ConstPtr& msg);
};

#endif
