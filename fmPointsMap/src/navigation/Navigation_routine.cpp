//=======================================================================
// Basic C++: class Navigation
// Specifications of class Navigation
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
* \file Navigation_routine.cpp
*
* \brief Implementation of class Navigation.
*/

#include "Navigation.h"

using namespace std;

//! Correct the angle
void Navigation::face_destination()
{
  Destination dest_turning = Calcul::turning_aim(_current_angle,
						  _current_position,
						  _current_destination);

  _update_angle = false;//DO NOT update the angle while turning
  ROS_INFO("[Navigation::face_destination] angle update ignored");
  if(dest_turning.id() == _CLOCKWISE_)
    _TURN_RIGHT_
  else//dest_turning.id() == _COUNTERWISE_
    _TURN_LEFT_

  //Turn until destination is reached
  ros::Rate loop_rate(_FREQUENCE_);
ROS_INFO("Before spinning!!%d",Calcul::distance(dest_turning,_current_position));
  while(ros::ok()
     && Calcul::distance(_current_position, dest_turning) <= _AREA_TURNING_)
  {
//calculating distance is always the same as _current_position.x()
ROS_INFO("current position %d,%d",_current_position.x(),_current_position.y());
ROS_INFO("Spinning!!%d", Calcul::distance(_current_position,dest_turning));
ROS_INFO("dest_turning%d,%d",dest_turning.x(),dest_turning.y());
    _motor_power_pub.publish(_motor_power_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  _update_angle = true;
  ROS_INFO("[Navigation::face_destination] angle update listened");

  //use current position and destination position to calculate the new_angle aimed
  // when you calculate the new_angle compare it with your current angle. 
  //if your new_angle is the same with your current_angle
  //then start moving forward.
//if (_current_angle!= _destination_angle)
  {
    //speed(1,-0.5);
    //set the lowest speed for turning 
    //turn until you find correct angle
    
  }
// else
  {
  //  speed(0,0);
   // stop turning
   //you are allowed to start moving.
  }

  
}

//! Go to the destination + correct the angle
void Navigation::go()
{
  //calculate intermediate points between the final destination and initial position
  face_destination();
  move_to_destination();
  //check your current destination and your angle
  //check if you arrived or not
  // if you arrived then stop.
  ROS_INFO("[Navigation::go] GOTO _current_destination:#%d(%d, %d)",
	  _current_destination.id(),
	  _current_destination.x(),
	  _current_destination.y()
    );
}

//! Make the robot reach the destination's area
void Navigation::move_to_destination()
{
  //moving to the destination,
  //if distance < = destination.distance-5cm {keep moving}
 //else {slow down}

}
