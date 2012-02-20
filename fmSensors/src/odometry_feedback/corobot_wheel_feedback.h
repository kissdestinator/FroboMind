#include "fmMsgs/encoder.h"
#include "fmMsgs/odometry.h"
#include "ros/ros.h"

#ifndef COROBOT_WHEEL_FEEDBACK_H_
#define COROBOT_WHEEL_FEEDBACK_H_

class CorobotWheelFeedback
{
private:

   int enc1_id;
   int enc2_id;

   int _PreviousLeftEncoderCounts;
   int _PreviousRightEncoderCounts;

   ros::Time last_time_left_encoder;
   ros::Time last_time_right_encoder;

   fmMsgs::odometry odo_msg;

public:

  ros::Subscriber enc1_sub; 
  ros::Subscriber enc2_sub; 

  ros::Publisher left_odometry_pub;
  ros::Publisher right_odometry_pub;

  CorobotWheelFeedback();
  void callbackHandlerEncoder1(const fmMsgs::encoderConstPtr& msg);
  void callbackHandlerEncoder2(const fmMsgs::encoderConstPtr& msg);
};

#endif
