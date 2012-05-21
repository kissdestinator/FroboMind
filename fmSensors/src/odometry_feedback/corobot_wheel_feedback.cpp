#include "corobot_wheel_feedback.h"

const double DistancePerCount = (M_PI*0.105)/(12*52*4); // Traveled distance per encodertick ((pi * wheeldiameter) / (ticks per wheelrotation))

CorobotWheelFeedback::CorobotWheelFeedback() {
	enc1_id = -1;
	enc2_id = -1;
	_PreviousLeftEncoderCounts = 0;
	_PreviousRightEncoderCounts = 0;
	_PreviousTimeLeftEncoder = ros::Time::now();
	_PreviousTimeRightEncoder = ros::Time::now();
	ticksCounterLeft = 0;
	ticksCounterRight = 0;
	ticksRight[0] = 0;
	ticksRight[1] = 0;
	ticksRight[2] = 0;
	ticksRight[3] = 0;
	ticksLeft[0] = 0;
	ticksLeft[1] = 0;
	ticksLeft[2] = 0;
	ticksLeft[3] = 0;

}

double CorobotWheelFeedback::calcTicksLeft(double ticks){
	ticksLeft[ticksCounterLeft] = ticks;
	double sum = ticksLeft[0] + ticksLeft[1] + ticksLeft[2] + ticksLeft[3];
//+ ticksLeft[5]+ ticksLeft[6]+ ticksLeft[7]+ ticksLeft[8]+ ticksLeft[9] + ticksLeft[10] + ticksLeft[11] + ticksLeft[12] + ticksLeft[13] + ticksLeft[14]+ ticksLeft[15]+ ticksLeft[16]+ ticksLeft[17]+ ticksLeft[18]+ ticksLeft[19];
	ticksCounterLeft++;
	if(ticksCounterLeft > 3)
		ticksCounterLeft = 0;
	return sum/4;
}

double CorobotWheelFeedback::calcTicksRight(double ticks){
	ticksRight[ticksCounterRight] = ticks;
	double sum = ticksRight[0] + ticksRight[1] + ticksRight[2] + ticksRight[3];
//+ ticksRight[5]+ ticksRight[6]+ ticksRight[7]+ ticksRight[8]+ ticksRight[9] + ticksRight[10] + ticksRight[11] + ticksRight[12] + ticksRight[13] + ticksRight[14]+ ticksRight[15]+ ticksRight[16]+ ticksRight[17]+ ticksRight[18]+ ticksRight[19] ;
	ticksCounterRight++;
	if(ticksCounterRight > 3)
		ticksCounterRight = 0;
	ROS_INFO("right: %f, ticks: %f", sum/4, ticks);
	return sum/4;
}

void CorobotWheelFeedback::callbackHandlerEncoder1(const fmMsgs::encoderConstPtr& msg)
{
	  if (enc2_id != -1 && enc1_id != -1)
	  {
		  odo_msg.header.stamp = ros::Time::now();
		  odo_msg_window.header.stamp = ros::Time::now();
		  if (enc1_id < enc2_id)
		  {
			  double temp = msg->encoderticks - _PreviousLeftEncoderCounts;
			  double temp_window = calcTicksLeft(msg->encoderticks - _PreviousLeftEncoderCounts);
			  odo_msg_window.position = -1 * (msg->encoderticks  * DistancePerCount);
			  odo_msg_window.speed = -1 * ((temp_window) * DistancePerCount)/ (msg->header.stamp - _PreviousTimeLeftEncoder).toSec();
			  left_odometry_pub_window.publish(odo_msg_window);
	
			  odo_msg.position = -1 * (msg->encoderticks  * DistancePerCount);
			  odo_msg.speed = -1 * ((temp) * DistancePerCount)/ (msg->header.stamp - _PreviousTimeLeftEncoder).toSec();
			  left_odometry_pub.publish(odo_msg);


			  _PreviousLeftEncoderCounts = msg->encoderticks;
			  _PreviousTimeLeftEncoder = msg->header.stamp;
		  }
		  else
		  {			  
			  double temp = msg->encoderticks - _PreviousRightEncoderCounts;
			  double temp_window = calcTicksRight(msg->encoderticks - _PreviousRightEncoderCounts);
			 
			  odo_msg_window.position = (msg->encoderticks  * DistancePerCount);
			  odo_msg_window.speed = ((temp_window) * DistancePerCount)/ (msg->header.stamp - _PreviousTimeRightEncoder).toSec();
			  right_odometry_pub_window.publish(odo_msg_window);

			  odo_msg.position = msg->encoderticks  * DistancePerCount;
			  odo_msg.speed = ((temp) * DistancePerCount)/ (msg->header.stamp - _PreviousTimeRightEncoder).toSec();
			  right_odometry_pub.publish(odo_msg);
			  _PreviousRightEncoderCounts = msg->encoderticks;
			  _PreviousTimeRightEncoder = msg->header.stamp;
		  }
	  }
	  else if (enc1_id == -1)
	  {
		  std::istringstream iss(msg->header.frame_id);
		  iss >> enc1_id;
	  }
}

void CorobotWheelFeedback::callbackHandlerEncoder2(const fmMsgs::encoderConstPtr& msg)
{
	  if (enc2_id != -1 && enc1_id != -1)
	  {
		  odo_msg.header.stamp = ros::Time::now();
		  odo_msg_window.header.stamp = ros::Time::now();
		  if (enc2_id < enc1_id)
		  {			  
			  double temp = msg->encoderticks - _PreviousLeftEncoderCounts;
			  double temp_window = calcTicksRight(msg->encoderticks - _PreviousLeftEncoderCounts);
			  odo_msg_window.position = -1 * (msg->encoderticks  * DistancePerCount);
			  odo_msg_window.speed = -1 * ((temp_window) * DistancePerCount)/ (msg->header.stamp - _PreviousTimeLeftEncoder).toSec();
			  left_odometry_pub_window.publish(odo_msg_window);
			  odo_msg.position = -1 * (msg->encoderticks  * DistancePerCount);
			  odo_msg.speed = -1 * ((temp) * DistancePerCount)/ (msg->header.stamp - _PreviousTimeLeftEncoder).toSec();
			  left_odometry_pub.publish(odo_msg);
			  _PreviousLeftEncoderCounts = msg->encoderticks;
			  _PreviousTimeLeftEncoder = msg->header.stamp;
		  }
		  else
		  {			  
			  double temp = msg->encoderticks - _PreviousRightEncoderCounts;
			  double temp_window = calcTicksRight(msg->encoderticks - _PreviousRightEncoderCounts);
			 
			  odo_msg_window.position = (msg->encoderticks  * DistancePerCount);
			  odo_msg_window.speed = ((temp_window) * DistancePerCount)/ (msg->header.stamp - _PreviousTimeRightEncoder).toSec();
			  right_odometry_pub_window.publish(odo_msg_window);

			  odo_msg.position = msg->encoderticks  * DistancePerCount;
			  odo_msg.speed = ((temp) * DistancePerCount)/ (msg->header.stamp - _PreviousTimeRightEncoder).toSec();
			  right_odometry_pub.publish(odo_msg);
			  _PreviousRightEncoderCounts = msg->encoderticks;
			  _PreviousTimeRightEncoder = msg->header.stamp;
		  }
	  }
	  else if (enc2_id == -1)
	  {
		  std::istringstream iss(msg->header.frame_id);
		  iss >> enc2_id;
	  }
}

