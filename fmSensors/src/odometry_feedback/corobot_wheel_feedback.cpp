#include "corobot_wheel_feedback.h"

const double DistancePerCount = (M_PI*0.105)/(12*52*4); // Traveled distance per encodertick ((pi * wheeldiameter) / (ticks per wheelrotation))

CorobotWheelFeedback::CorobotWheelFeedback() {
	enc1_id = -1;
	enc2_id = -1;
	_PreviousLeftEncoderCounts = 0;
	_PreviousRightEncoderCounts = 0;
	_PreviousTimeLeftEncoder = ros::Time::now();
	_PreviousTimeRightEncoder = ros::Time::now();
}

void CorobotWheelFeedback::callbackHandlerEncoder1(const fmMsgs::encoderConstPtr& msg)
{
	  if (enc1_id == -1)
	  {
		  std::istringstream iss(msg->header.frame_id);
		  iss >> enc1_id;
	  }
	  if (enc2_id != -1 && enc1_id != -1)
	  {
		  odo_msg.header.stamp = ros::Time::now();
		  if (enc1_id < enc2_id)
		  {
			  odo_msg.position = -1 * (msg->encoderticks * DistancePerCount);
			  odo_msg.speed = -1 * ((msg->encoderticks - _PreviousLeftEncoderCounts) * DistancePerCount)/ (msg->header.stamp - _PreviousTimeLeftEncoder).toSec();
			  left_odometry_pub.publish(odo_msg);
			  _PreviousLeftEncoderCounts = msg->encoderticks;
			  _PreviousTimeLeftEncoder = msg->header.stamp;
		  }
		  else
		  {
			  odo_msg.position = msg->encoderticks * DistancePerCount;
			  odo_msg.speed = ((msg->encoderticks - _PreviousRightEncoderCounts) * DistancePerCount)/ (msg->header.stamp - _PreviousTimeRightEncoder).toSec();
			  right_odometry_pub.publish(odo_msg);
			  _PreviousRightEncoderCounts = msg->encoderticks;
			  _PreviousTimeRightEncoder = msg->header.stamp;
		  }
	  }
}

void CorobotWheelFeedback::callbackHandlerEncoder2(const fmMsgs::encoderConstPtr& msg)
{
	  if (enc2_id == -1)
	  {
	    std::istringstream iss(msg->header.frame_id);
	    iss >> enc2_id;
	  }
	  if (enc2_id != -1 && enc1_id != -1)
	  {
		  odo_msg.header.stamp = ros::Time::now();
		  if (enc2_id < enc1_id)
		  {
			  odo_msg.position = -1 * (msg->encoderticks * DistancePerCount);
			  odo_msg.speed = -1 * ((msg->encoderticks - _PreviousLeftEncoderCounts) * DistancePerCount)/ (msg->header.stamp - _PreviousTimeLeftEncoder).toSec();
			  left_odometry_pub.publish(odo_msg);
			  _PreviousLeftEncoderCounts = msg->encoderticks;
			  _PreviousTimeLeftEncoder = msg->header.stamp;
		  }
		  else
		  {
			  odo_msg.position = msg->encoderticks * DistancePerCount;
			  odo_msg.speed = ((msg->encoderticks - _PreviousRightEncoderCounts) * DistancePerCount)/ (msg->header.stamp - _PreviousTimeRightEncoder).toSec();
			  right_odometry_pub.publish(odo_msg);
			  _PreviousRightEncoderCounts = msg->encoderticks;
			  _PreviousTimeRightEncoder = msg->header.stamp;
		  }
	  }
}

