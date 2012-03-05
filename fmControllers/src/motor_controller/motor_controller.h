#include "fmMsgs/desired_speed.h"
#include "fmMsgs/motor_power.h"
#include "fmMsgs/odometry.h"
#include "ros/ros.h"
#include "pid_regulator.h"

#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

class MotorController
{
private:

   fmMsgs::motor_power power_msg;

   float target_speed_left;
   float target_speed_right;

   float max_speed;

   PIDRegulator pid_regulator_left;
   PIDRegulator pid_regulator_right;

public:

  ros::Subscriber target_speed_sub;
  ros::Subscriber left_odo_sub;
  ros::Subscriber right_odo_sub;

  ros::Publisher motor_power_pub;

  MotorController();
  MotorController(double p_left, double i_left, double d_left, double p_right, double i_right, double d_right);
  void desiredSpeedHandler(const fmMsgs::desired_speedConstPtr& msg);
  void leftMotorHandler(const fmMsgs::odometryConstPtr& msg);
  void rightMotorHandler(const fmMsgs::odometryConstPtr& msg);

  void setMaxSpeed(float speed);
};

#endif
