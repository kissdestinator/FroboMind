#include "motor_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

MotorController::MotorController() {
	power_msg.power_left = 0;
	power_msg.power_right = 0;
}

MotorController::MotorController(double p_left, double i_left, double d_left, double p_right, double i_right, double d_right) {
	power_msg.power_left = 0;
	power_msg.power_right = 0;

	pid_regulator_left = PIDRegulator(p_left,i_left,d_left);
	pid_regulator_right = PIDRegulator(p_right,i_right,d_right);
}

void MotorController::desiredSpeedHandler(const fmMsgs::desired_speedConstPtr& msg)
{
	target_speed_left = msg->speed_left * max_speed;
	target_speed_right = msg->speed_right * max_speed;
}

void MotorController::leftMotorHandler(const fmMsgs::odometryConstPtr& msg)
{
	power_msg.power_left += pid_regulator_left.update(msg->speed,target_speed_left) / max_speed;
	motor_power_pub.publish(power_msg);
}

void MotorController::rightMotorHandler(const fmMsgs::odometryConstPtr& msg)
{
	power_msg.power_right += pid_regulator_right.update(msg->speed,target_speed_right) / max_speed;
	motor_power_pub.publish(power_msg);
}

void MotorController::setMaxSpeed(float speed)
{
	max_speed = speed;
}

