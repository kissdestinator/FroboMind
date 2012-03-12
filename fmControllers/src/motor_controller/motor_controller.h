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

   double target_speed_left;
   double target_speed_right;

   double motor_power_left;
   double motor_power_right;

   ros::Time last_time_left;
   ros::Time last_time_right;

   double max_acceleration;
   double max_deacceleration;
   double max_speed;

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
  void setMaxAcceleration(float acceleration);
  void setMaxDeacceleration(float deacceleration);
};

#endif
