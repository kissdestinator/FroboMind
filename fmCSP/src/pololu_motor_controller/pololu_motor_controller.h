#include "fmMsgs/serial.h"
#include "fmMsgs/motor_power.h"
#include "ros/ros.h"

#ifndef POLOLU_MOTOR_CONTROLLER_H_
#define POLOLU_MOTOR_CONTROLLER_H_

class PololuMotorController
{
private:

   fmMsgs::serial serial_msg;

public:

  ros::Subscriber pololu_sub;

  ros::Publisher pololu_pub;

  PololuMotorController();
  void callbackHandler(const fmMsgs::motor_powerConstPtr& msg);
};

#endif
