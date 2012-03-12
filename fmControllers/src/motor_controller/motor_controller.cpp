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
	max_deacceleration = 0;
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
	double pid_update_value = pid_regulator_left.update(msg->speed,target_speed_left);

	if (pid_update_value > 0)
	{
		if (pid_update_value > max_acceleration * dt && max_acceleration != 0)
			pid_update_value = max_acceleration * dt;
		else if (pid_update_value < -max_deacceleration * dt && max_deacceleration != 0)
			pid_update_value = -max_deacceleration * dt;
	}
	else
	{
		if (pid_update_value > -max_acceleration * dt && max_acceleration != 0)
			pid_update_value = -max_acceleration * dt;
		else if (pid_update_value < max_deacceleration * dt && max_deacceleration != 0)
			pid_update_value = max_deacceleration * dt;
	}

	motor_power_left += pid_update_value / max_speed;

	if (motor_power_left > 1)
		motor_power_left = 1;
	else if (motor_power_left < -1)
		motor_power_left = -1;

	power_msg.power_left = motor_power_left;
	motor_power_pub.publish(power_msg);
}

void MotorController::rightMotorHandler(const fmMsgs::odometryConstPtr& msg)
{
	double dt = (ros::Time::now() - last_time_right).toSec();
	last_time_right = ros::Time::now();
	double pid_update_value = pid_regulator_right.update(msg->speed,target_speed_right);

	if (pid_update_value > 0)
	{
		if (pid_update_value > max_acceleration * dt && max_acceleration != 0)
			pid_update_value = max_acceleration * dt;
		else if (pid_update_value < -max_deacceleration * dt && max_deacceleration != 0)
			pid_update_value = -max_deacceleration * dt;
	}
	else
	{
		if (pid_update_value > -max_acceleration * dt && max_acceleration != 0)
			pid_update_value = -max_acceleration * dt;
		else if (pid_update_value < max_deacceleration * dt && max_deacceleration != 0)
			pid_update_value = max_deacceleration * dt;
	}

	motor_power_right += pid_update_value / max_speed;

	if (motor_power_right > 1)
		motor_power_right = 1;
	else if (motor_power_right < -1)
		motor_power_right = -1;

	power_msg.power_right = motor_power_right;
	motor_power_pub.publish(power_msg);
}

void MotorController::setMaxSpeed(double speed)
{
	max_speed = speed;
}

void MotorController::setMaxAcceleration(double acceleration)
{
	max_acceleration = acceleration;
}

void MotorController::setMaxDeacceleration(double deacceleration)
{
	max_deacceleration = deacceleration;
}

