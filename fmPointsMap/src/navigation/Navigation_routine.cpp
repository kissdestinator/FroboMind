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
  
}

//! Make the robot reach the destination's area
void Navigation::move_to_destination()
{
  //moving to the destination,
  //if distance < = destination.distance-5cm {keep moving}
 //else {slow down}

}