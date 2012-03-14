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
   double max_deceleration;
   double max_speed;

   PIDRegulator pid_regulator_left;
   PIDRegulator pid_regulator_right;

public:

  ros::Subscriber target_speed_sub;
  ros::Subscriber left_odo_sub;
  ros::Subscriber right_odo_sub;

  ros::Publisher motor_power_pub;

  MotorController(double p_left, double i_left, double d_left, double p_right, double i_right, double d_right);

  void desiredSpeedHandler(const fmMsgs::desired_speedConstPtr& msg);
  void leftMotorHandler(const fmMsgs::odometryConstPtr& msg);
  void rightMotorHandler(const fmMsgs::odometryConstPtr& msg);

  void setMaxSpeed(double speed) { max_speed = speed; };
  void setMaxAcceleration(double acceleration) { max_acceleration = acceleration; };
  void setMaxDeacceleration(double deceleration) { max_deceleration = deceleration; };

  void setPleft(double p) { pid_regulator_left.setP(p); };
  void setIleft(double i) { pid_regulator_left.setI(i); };
  void setDleft(double d) { pid_regulator_left.setD(d); };

  void setPright(double p) { pid_regulator_right.setP(p); };
  void setIright(double i) { pid_regulator_right.setI(i); };
  void setDright(double d) { pid_regulator_right.setD(d); };

  double getPleft(void) { return pid_regulator_left.getP(); };
  double getIleft(void) { return pid_regulator_left.getI(); };
  double getDleft(void) { return pid_regulator_left.getD(); };

  double getPright(void) { return pid_regulator_right.getP(); };
  double getIright(void) { return pid_regulator_right.getI(); };
  double getDright(void) { return pid_regulator_right.getD(); };
};

#endif
