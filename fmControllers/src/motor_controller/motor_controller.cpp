#include "motor_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

MotorController::MotorController(double p_left, double i_left, double d_left, double p_right, double i_right, double d_right) {
	power_msg.power_left = 0;
	power_msg.power_right = 0;

	pid_regulator_left = PIDRegulator(p_left,i_left,d_left);
	pid_regulator_right = PIDRegulator(p_right,i_right,d_right);

	max_speed = 1;
	max_acceleration = 0;
	max_deceleration = 0;
}

void MotorController::desiredSpeedHandler(const fmMsgs::desired_speedConstPtr& msg)
{
	target_speed_left = msg->speed_left * max_speed;
	target_speed_right = msg->speed_right * max_speed;
}

void MotorController::leftMotorHandler(const fmMsgs::odometryConstPtr& msg)
{
	double dt = (ros::Time::now() - last_time_left).toSec();
	last_time_left = ros::Time::now();
	double speed_left = target_speed_left;

	//implementation of max acceleration and max deceleration
	if (msg->speed > 0) // Driving forwards
	{
		if (target_speed_left - msg->speed > max_acceleration * dt && max_acceleration != 0) // accelerating
			speed_left = max_acceleration * dt + msg->speed;
		else if (target_speed_left - msg->speed < -max_deceleration * dt && max_deceleration != 0) // Decelerating
			speed_left = -max_deceleration * dt + msg->speed;
	}
	else if (msg->speed < 0)// Driving backwards
	{
		if (target_speed_left + msg->speed < -max_acceleration * dt && max_acceleration != 0) // accelerating
			speed_left = -max_acceleration * dt + msg->speed;
		else if (target_speed_left - msg->speed > max_deceleration * dt && max_deceleration != 0) // decelerating
			speed_left = max_deceleration * dt + msg->speed;
	}

	// Calculate the motor power: PID update and directly forwarding the target speed and in the end normalizing by the max_speed
	motor_power_left = (pid_regulator_left.update(msg->speed,speed_left) + speed_left) / max_speed;

	// making sure the motor power does not exceed 1 or -1
	if (motor_power_left > 1)
		motor_power_left = 1;
	else if (motor_power_left < -1)
		motor_power_left = -1;

	// publish the motor power
	power_msg.power_left = motor_power_left;
	motor_power_pub.publish(power_msg);
}

void MotorController::rightMotorHandler(const fmMsgs::odometryConstPtr& msg)
{
	double dt = (ros::Time::now() - last_time_right).toSec();
	last_time_right = ros::Time::now();
	double speed_right = target_speed_right;

	//implementation of max acceleration and max deceleration
	if (msg->speed > 0) // Driving forwards
	{
		if (target_speed_right - msg->speed > max_acceleration * dt && max_acceleration != 0) // accelerating
			speed_right = max_acceleration * dt + msg->speed;
		else if (target_speed_right - msg->speed < -max_deceleration * dt && max_deceleration != 0) // Decelerating
			speed_right = -max_deceleration * dt + msg->speed;
	}
	else if (msg->speed < 0)// Driving backwards
	{
		if (target_speed_right + msg->speed < -max_acceleration * dt && max_acceleration != 0) // accelerating
			speed_right = -max_acceleration * dt + msg->speed;
		else if (target_speed_right - msg->speed > max_deceleration * dt && max_deceleration != 0) // decelerating
			speed_right = max_deceleration * dt + msg->speed;
	}

	// Calculate the motor power: PID update and directly forwarding the target speed and in the end normalizing by the max_speed
	motor_power_right = (pid_regulator_left.update(msg->speed,speed_right) + speed_right) / max_speed;

	// making sure the motor power does not exceed 1 or -1
	if (motor_power_right > 1)
		motor_power_right = 1;
	else if (motor_power_right < -1)
		motor_power_right = -1;

	// publish the motor power
	power_msg.power_right = motor_power_right;
	motor_power_pub.publish(power_msg);
}

