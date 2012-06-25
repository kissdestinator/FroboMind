#include "geometry_msgs/TwistStamped.h"
#include "fmMsgs/motor_power.h"
#include "fmMsgs/odometry.h"
#include "ros/ros.h"
#include "pid_regulator.h"
#include "fmMsgs/warhorse_state.h"

#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

class MotorController
{
private:

   fmMsgs::motor_power power_msg;

   double target_speed_left;
   double target_speed_right;

   double last_target_speed_left;
   double last_target_speed_right;

   double motor_power_left;
   double motor_power_right;

   ros::Time last_time_left;
   ros::Time last_time_right;

   double max_acceleration;
   double max_deceleration;
   double max_speed;

   PIDRegulator pid_regulator_left;
   PIDRegulator pid_regulator_right;

   double maxAcceleration(const double& target_speed, const double& last_target_speed, ros::Time& last_time);
   void calcSpeed(const geometry_msgs::TwistStampedConstPtr& msg);
   void zeroSpeed();

public:

  ros::Subscriber navigation_speed_sub;
  ros::Subscriber wii_speed_sub;
  ros::Subscriber warhorse_state_sub;
  ros::Subscriber left_odo_sub;
  ros::Subscriber right_odo_sub;

  ros::Publisher motor_power_pub;

  fmMsgs::warhorse_state warhorse_state;

  MotorController(double p_left, double i_left, double d_left, double p_right, double i_right, double d_right);

  void leftMotorHandler(const fmMsgs::odometryConstPtr& msg);
  void rightMotorHandler(const fmMsgs::odometryConstPtr& msg);

  void navigationSpeedHandler(const geometry_msgs::TwistStampedConstPtr& msg);
  void wiiSpeedHandler(const geometry_msgs::TwistStampedConstPtr& msg);
  void stateHandler(const fmMsgs::warhorse_stateConstPtr& msg);

  void setMaxSpeed(double speed) { max_speed = speed; };
  void setMaxAcceleration(double acceleration) { max_acceleration = acceleration; };
  void setMaxDeacceleration(double deceleration) { max_deceleration = deceleration; };

  double getMaxSpeed(void) { return max_speed; };
  double getMaxAcceleration(void) { return max_acceleration; };
  double getMaxDeceleration(void) { return max_deceleration; };

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

  void setIWindupLimitLeft(double limit) {pid_regulator_left.setIWindupLimit(limit);};
  void setIWindupLimitRight(double limit) {pid_regulator_right.setIWindupLimit(limit);};
};

#endif
