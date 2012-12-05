//=======================================================================
// Basic C++: class Navigation
// Specifications of class Navigation
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
* \file Navigation.cpp
*
* \brief Implementation of class Navigation.
*/

#include "Navigation.h"
#include "Calcul.h"

#define X_HOME 400
#define Y_HOME 1200
//each time the robot move to 30 mm the angle is updated
#define _SMALL_DIST_ 	30
#define _UPDATE_ 	_update==true
#define _TURNING_ 	_turning==true
#define _NO_BACKWARD_ 	_update==false
#define _IS_NOT_GOOD_ 	> 0
#define _FREQUENCE_ 	500
#define _AREA_ 		300
#define _AREA_TURNING_	50

using namespace std;

//-----------------------------------------------------------------------
// Accessors
//-----------------------------------------------------------------------
/*!
* Return a default point (go Home) for now.
*/
Destination Navigation::goal() const
{
  Destination home;
  home.set(X_HOME, Y_HOME);
  return home;
}

//-----------------------------------------------------------------------
// Functions for the navigation itself
//-----------------------------------------------------------------------

/*!
 * Make the initialisation with the angle of the robot and so on
 * Do not make it in the constructor to allow the user to choose
 * the initialisation date
 * *This function will make the robot move*
 */
void Navigation::initialisation() {
  speed(0.2,0.2);

  _current_destination = NULL;
  _current_destination_turning = NULL;
  _current_angle = NULL;

  ros::Rate loop_rate(_FREQUENCE_);
  while (ros::ok() && _current_angle == -1)
  {
    _motor_power_pub.publish(_motor_power_msg);
    ros::spin();
    loop_rate.sleep();
  }
  _update = false;
  go_back();
  _update = true;
}

int Navigation::check_default_value()
{
  int error = 0;
  if(_current_position.is_default())
  {
    ROS_ERROR("current position,");
    error = 1;
  }
  if(_destination == -1)
  {
    ROS_ERROR("destination,");
    error = 10;
  }
  if(_current_angle == -1)
  {
    ROS_ERROR("current angle,");
    error = 100;
  }
  if(error > 0)
    ROS_ERROR(" hasn't been set!");
  return error;
}

//!< Set the speed to the msg *DOES NOT PUBLISH*
void Navigation::speed(double right, double left)
{
  _motor_power_msg.power_right = (right > -1) ? right : -1;
  _motor_power_msg.power_right = (right <  1) ? right :  1;
  _motor_power_msg.power_left  = (left  > -1) ?  left : -1;
  _motor_power_msg.power_left  = (left  >  1) ?  left :  1;
}

//! Make the robot return 3 cm backward without updating the angle
void Navigation::go_back()
{
  speed(-0.2,-0.2);

  ros::Rate loop_rate(_FREQUENCE_);
  while (ros::ok() || !moved())
  {
    _motor_power_pub.publish(_motor_power_msg);
    ros::spin();
    loop_rate.sleep();
  }
}

/*!
* Update the angle value.
* *This function will make the robot move*
*/
void Navigation::update_angle()
{
  if(moved() && _UPDATE_ && !_TURNING_)
  {
    _current_angle = Calcul::angle(_old_position, _current_position);
    _old_position = _current_position;
    ROS_INFO("current angle updated:%g", _current_angle);
  }
}

//!Update the current position
void Navigation::update_position(int x, int y)
{
  _current_position.set(x, y);
}

bool Navigation::moved() const
{
  return (_SMALL_DIST_ >= Calcul::distance(_old_position,
					    _current_position));
}

/*!
* Update the position and the angle if the robot moved enough
*/
void Navigation::update(const fmMsgs::gtps::ConstPtr& msg)
{
  update_position(msg->x, msg->y);
  update_angle();
}

//! Method called at each message publish from web node.
void Navigation::set_new_destination(const fmMsgs::web:: ConstPtr& msg)
{
  if(_listening) {
  Destination new_dest = _map.find_destination(msg->id);
  if(new_dest == NULL)
    _current_destination = Destination(_current_position);
  else
    _current_destination = new_dest;
  ROS_INFO("Current destination updated: (%d,%d).",
	   _current_destination.x(),
	   _current_destination.y());
  }
}

/*!
 * Calculate the angle between two points.
 * If 1st parameter is NULL, it takes the current position
 */
int Navigation::distance_to_destination()
{
    return (_current_destination == NULL) ?
      int(Calcul::distance(_current_position,
			   _current_destination))
      : 0 ;//if no destination return 0
}

//! Correct the angle
void Navigation::face_destination()
{
  _turning = true;
  double turn = 1;
  double stop = -0.5;
  _current_destination_turning = Calcul::turning_aim(_current_angle,
						      _current_position,
						      _current_destination);
  if(_current_destination_turning.id() == _CLOCKWISE_)
    while(distance_to_destination() > _AREA_TURNING_)
      speed(stop, turn);//stop the right move the left
  else
    while(distance_to_destination() > _AREA_TURNING_)
      speed(turn, stop);//stop the left move the right

  _turning = false;
}

//! Go to the destination + correct the angle
void Navigation::go()
{
  
  
}

//! Make the robot reach the destination's area
void Navigation::move_to_destination()
{
  _listening = false;//one destination a time
  face_destination();//Correct the angle
  go();//Go forward to the destination
}

//! Start the routine
void Navigation::start()
{
  initialisation();
  if(check_default_value() _IS_NOT_GOOD_)
  {
    ROS_ERROR("One -at least- value is unusabled. *Exit the node*");
    exit -1;
  }
    
  _listening = true; // we start listening to te web node
  ROS_INFO("Navigation.cpp entering start():while loop");
  while (ros::ok())
  {
    _listening = true;
    if(distance_to_destination() > _AREA_)
      move_to_destination();
  }
}