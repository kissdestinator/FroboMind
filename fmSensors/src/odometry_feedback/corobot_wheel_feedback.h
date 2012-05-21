#include "fmMsgs/encoder.h"
#include "fmMsgs/odometry.h"
#include "ros/ros.h"
#include <queue>


#ifndef COROBOT_WHEEL_FEEDBACK_H_
#define COROBOT_WHEEL_FEEDBACK_H_

class CorobotWheelFeedback
{
private:

   int enc1_id;
   int enc2_id;

   int _PreviousLeftEncoderCounts;
   int _PreviousRightEncoderCounts;

   ros::Time _PreviousTimeLeftEncoder;
   ros::Time _PreviousTimeRightEncoder;

   fmMsgs::odometry odo_msg;
   fmMsgs::odometry odo_msg_window;

   double ticksLeft[5];
   double ticksRight[5];

   double calcTicksLeft(double ticks);
   double calcTicksRight(double ticks);

   int ticksCounterLeft;
   int ticksCounterRight;
   

public:

  ros::Subscriber enc1_sub; 
  ros::Subscriber enc2_sub; 

  ros::Publisher left_odometry_pub;
  ros::Publisher right_odometry_pub;
  ros::Publisher left_odometry_pub_window;
  ros::Publisher right_odometry_pub_window;

  CorobotWheelFeedback();
  void callbackHandlerEncoder1(const fmMsgs::encoderConstPtr& msg);
  void callbackHandlerEncoder2(const fmMsgs::encoderConstPtr& msg);
};

#endif
