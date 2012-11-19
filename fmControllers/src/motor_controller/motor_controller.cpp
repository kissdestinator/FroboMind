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

	warhorse_state.drive_state = warhorse_state.TASK1LEFT;
	warhorse_state.task_state = warhorse_state.DRIVE;

ROS_DEBUG("motor controll init");
}

void MotorController::stateHandler(const fmMsgs::warhorse_stateConstPtr& msg)
{
	warhorse_state.drive_state = msg->drive_state;
	warhorse_state.task_state = msg->task_state;
}

void MotorController::navigationSpeedHandler(const geometry_msgs::TwistStampedConstPtr& msg)
{
ROS_DEBUG("speed handler");
	if (warhorse_state.task_state != warhorse_state.MANUAL_DRIVE)
		if (warhorse_state.drive_state == warhorse_state.DRIVE)
			calcSpeed(msg);
		else
			zeroSpeed();
}

void MotorController::wiiSpeedHandler(const geometry_msgs::TwistStampedConstPtr& msg)
{
	if (warhorse_state.task_state == warhorse_state.MANUAL_DRIVE)
	{
		if (warhorse_state.drive_state == warhorse_state.DRIVE)
			calcSpeed(msg);
		else
			zeroSpeed();
	}
}

void MotorController::zeroSpeed()
{
	target_speed_left = 0;
	target_speed_right = 0;
}

void MotorController::calcSpeed(const geometry_msgs::TwistStampedConstPtr& msg)
{

	double W = 0.24; //length from center to meter
	double vel_right = msg->twist.linear.x - ( W * msg->twist.angular.z );
	double vel_left =  msg->twist.linear.x + ( W * msg->twist.angular.z );

	// Normalize velocities
	if (vel_right > 1)
	{
		vel_left /= vel_right;
		vel_right = 1;
	}
	if (vel_left > 1)
	{
		vel_right /= vel_left;
		vel_left = 1;
	}
	if (vel_right < -1)
	{
		vel_left /= abs(vel_right);
		vel_right = -1;
	}
	if (vel_left < -1)
	{
		vel_right /= abs(vel_left);
		vel_left = -1;
	}

	target_speed_left = vel_left * max_speed;
	target_speed_right = vel_right * max_speed;
	ROS_INFO("Target_speed_left: %f , target_speed_right: %f", target_speed_left, target_speed_right);
}

double MotorController::maxAcceleration(const double& target_speed, const double& last_target_speed, ros::Time& last_time)
{
	double dt = (ros::Time::now() - last_time).toSec();
	last_time = ros::Time::now();
	double r = target_speed;
	double dSpeed = target_speed - last_target_speed;

	//implementation of max acceleration and max deceleration
	if (last_target_speed >= 0) // Driving forwards
	{
		if (dSpeed > max_acceleration * dt && max_acceleration != 0) // accelerating
			r = last_target_speed + (max_acceleration * dt);
		else if (dSpeed < -max_deceleration * dt && max_deceleration != 0) // Decelerating
			r = last_target_speed - (max_deceleration * dt);
	}
	else // Driving backwards
	{
		if (dSpeed < -max_acceleration * dt && max_acceleration != 0) // accelerating
			r = last_target_speed - (max_acceleration * dt);
		else if (dSpeed > max_deceleration * dt && max_deceleration != 0) // Decelerating
			r = last_target_speed + (max_deceleration * dt);
	}

	return r;
}

void MotorController::leftMotorHandler(const fmMsgs::odometryConstPtr& msg)
{
	// Set last_target_speed for next loop using the maxAcceleration function
	last_target_speed_left = maxAcceleration(target_speed_left,last_target_speed_left,last_time_left);

	// Calculate the motor power: PID update and directly forwarding the target speed and in the end normalizing by the max_speed
	double pid_error = pid_regulator_left.update(msg->speed,last_target_speed_left);
	motor_power_left = (pid_error + last_target_speed_left) / max_speed;

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
	// Set last_target_speed for next loop using the maxAcceleration function
	last_target_speed_right = maxAcceleration(target_speed_right,last_target_speed_right,last_time_right);

	// Calculate the motor power: PID update and directly forwarding the target speed and in the end normalizing by the max_speed
	double pid_error = pid_regulator_right.update(msg->speed,last_target_speed_right);
	motor_power_right = (pid_error + last_target_speed_right) / max_speed;

	// making sure the motor power does not exceed 1 or -1
	if (motor_power_right > 1)
		motor_power_right = 1;
	else if (motor_power_right < -1)
		motor_power_right = -1;

	// publish the motor power
	power_msg.power_right = motor_power_right;
	motor_power_pub.publish(power_msg);
}

