/****************************************************************************
 # Inrow vehicle detector
 # Copyright (c) 2012 Jeppe Pedersen <jepe009@student.sdu.dk>
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.
 #
 *****************************************************************************
 # File: inrow_vehicle_detector.cpp
 # Purpose: place vehicle in row.
 # Project: Field Robot Event 2012 - Warhorse
 # Author: Jeppe Pedersen <baronjeppe@gmail.com>
 # Created: Mar 15 2012, Jeppe Pedersen, Source written
 ****************************************************************************/

#include "lidar_navigator.h"

#define DEBUG 1
#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI

LidarNavigator::LidarNavigator()
{
}

void LidarNavigator::positionCallback(const fmMsgs::vehicle_coordinateConstPtr& position)
{
	turn_angle -= position->th - old_th;
	old_th = position->th;
}

void LidarNavigator::processLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan )
{
	std::vector<double> ranges;
	int LRS_size = laser_scan->get_ranges_size();

	if (laser_inverted)
		for (int i = LRS_size-1; i >= 0; i--)
			ranges.push_back(laser_scan->ranges[i]);
	else
		for (int i = 0; i > LRS_size; i++)
			ranges.push_back(laser_scan->ranges[i]);

	std::vector<hole> holes;

	// run through the laser scan to search for "holes"
	for (int i = 0; i < 2*LRS_size; i++)
	{
		// find the first point with no obstacles in the way
		if (ranges[i] > nav_range || ranges[i] < min_range)
		{
			hole h;
			// set left angle to the last point with an obstacle
			h.left_angle = (i-1) % LRS_size;

			// find the next obstacle
			int j = i;
			while((ranges[j%LRS_size] > nav_range || ranges[j%LRS_size] < min_range) && j < 2*LRS_size)
				j++;

			// setting the right angle to the next obstacle found
			h.right_angle = j%LRS_size;

			// calculating the angle between the two points taking in to account that the scan is cyclic
			if (h.left_angle > h.right_angle)
				h.angle = LRS_size-h.left_angle + h.right_angle;
			else
				h.angle = h.right_angle - h.left_angle;

			// calculating the width between the two points - to check if the car fits between
			if (h.angle < LRS_size/2)
				h.width = MIN(ranges[h.right_angle],ranges[h.left_angle]) * sin((h.angle/2)*2*M_PI/LRS_size) * 2.0;
			else
				h.width = 1;

			// calculating the center angle of the hole
			h.center_angle = (h.left_angle + h.angle/2.0);
			if (h.center_angle >= LRS_size)
				h.center_angle -= LRS_size;

			// if the width is large enough the "hole" is accepted and pushed back in our vector
			if (h.width > min_clearance_width)
			{
				if (DEBUG)
					ROS_INFO("Hole: Left: %d, Right: %d, Center: %.2f, width: %.3f, angle: %.3f, LeftRange: %.3f, RightRange: %.3f",h.left_angle, h.right_angle, h.center_angle, h.width, h.angle,ranges[h.left_angle],ranges[h.right_angle]);
				holes.push_back(h);
			}
		}
	}
	// if any holes was found we start searching for the most desirable to go through
	if (holes.size() > 0)
	{
		// we find the hole closest to our goal
		int index = 0;
		for (int i = 0; i < holes.size(); i++)
		{
			if (MIN(MIN(abs(holes[i].center_angle - desired_heading),abs(holes[i].center_angle - LRS_size - desired_heading)),MIN(abs(holes[i].center_angle - desired_heading),abs(holes[i].center_angle + LRS_size - desired_heading))) < MIN(MIN(abs(holes[index].center_angle - desired_heading),abs(holes[index].center_angle - LRS_size - desired_heading)),MIN(abs(holes[index].center_angle - desired_heading),abs(holes[index].center_angle + LRS_size - desired_heading))))
				index = i;
		}
		hole h = holes[index];

		// the angles at which the car can pass the hole is calculated
		h.allowed_left_angle = h.left_angle + ( min_clearance_width/2 * h.angle/h.width );
		if (h.allowed_left_angle > LRS_size)
			h.allowed_left_angle -= LRS_size;
		h.allowed_right_angle = h.right_angle - ( min_clearance_width/2 * h.angle/h.width );
		if (h.allowed_right_angle < 0)
			h.allowed_right_angle += LRS_size;

		// the optimum angle is calculated and converted to radians
		turn_angle = calcTurnAngle(h,desired_heading,LRS_size);

		// Calculate and publish the vehicle speed
		calcAndPublishSpeed(h,desired_heading,turn_angle,max_velocity);
	}
	else
	{
		ROS_INFO("No holes wide enough found..! - setting speed to zero");
		hole h;
		calcAndPublishSpeed(h,desired_heading,0,0);
	}

}

void LidarNavigator::calcAndPublishSpeed(const hole& h, double goal, double turn_angle, double velocity)
{
	double vel = 0;
	double ang_vel = 0;

	// Calculate velocity
	if (abs(turn_angle) < 15 * DEG2RAD)
		vel = velocity;
	else if (abs(turn_angle) < 90 * DEG2RAD)
		vel = velocity - ((abs(turn_angle)*RAD2DEG - 15) / 75);

	// Calculate Angular velocity
	ang_vel = turn_angle * K_ang_vel;

	publishVisualization(h,turn_angle);

	geometry_msgs::TwistStamped twist;
	twist.twist.linear.x = vel;
	twist.twist.angular.z = ang_vel;
	velocity_pub.publish(twist);

	ROS_INFO("LeftAngle: %d, RightAngle: %d, LeftAllowedAngle: %.2f, RightAllowedAngle: %.2f, Width: %.2f, TurnAngle: %.2f - Vel: %.2f, AngVel: %.2f",h.left_angle, h.right_angle, h.allowed_left_angle, h.allowed_right_angle, h.width, turn_angle, vel, ang_vel);
}

// finds the optimum turn angle
double LidarNavigator::calcTurnAngle(const hole& h, double goal, int LRS_size)
{
	double r = 0;
	std::string output;

	// finds the best posible turn angle based on the hole and goal
	if (goal < h.allowed_right_angle && goal > h.allowed_left_angle && h.allowed_left_angle < h.allowed_right_angle)
	{
		r = goal;
		if(DEBUG)
			ROS_INFO("1: Goal allowed");
	}
	else if (goal < h.allowed_right_angle && h.allowed_left_angle > h.allowed_right_angle)
	{
		r = goal;
		if(DEBUG)
			ROS_INFO("2: Goal allowed");
	}
	else if (goal > h.allowed_left_angle && h.allowed_left_angle > h.allowed_right_angle)
	{
		r = goal;
		if(DEBUG)
			ROS_INFO("3: Goal allowed");
	}
	else if (abs(h.allowed_left_angle - goal) < abs(h.allowed_right_angle - goal) && h.allowed_left_angle > h.allowed_right_angle)
	{
		r = h.allowed_left_angle;
		if(DEBUG)
			ROS_INFO("4: Allowed left angle chosen");
	}
	else if (abs(h.allowed_right_angle - goal) < abs(h.allowed_left_angle - goal) && h.allowed_left_angle > h.allowed_right_angle)
	{
		r = h.allowed_right_angle;
		if(DEBUG)
			ROS_INFO("5: Allowed right angle chosen");
	}
	else if (abs(goal-LRS_size-h.allowed_left_angle) < abs(goal-h.allowed_right_angle) && h.allowed_left_angle < h.allowed_right_angle)
	{
		r = h.allowed_left_angle;
		if(DEBUG)
			ROS_INFO("6: Allowed left angle chosen");
	}
	else if (abs(goal+LRS_size-h.allowed_right_angle) < abs(goal-h.allowed_left_angle) && h.allowed_left_angle < h.allowed_right_angle)
	{
		r = h.allowed_right_angle;
		if(DEBUG)
			ROS_INFO("7: Allowed right angle chosen");
	}
	else if (abs(goal-h.allowed_left_angle) < abs(goal-h.allowed_right_angle) && h.allowed_left_angle < h.allowed_right_angle)
	{
		r = h.allowed_left_angle;
		if(DEBUG)
			ROS_INFO("8: Allowed left angle chosen");
	}
	else if (abs(goal-h.allowed_right_angle) < abs(goal-h.allowed_left_angle) && h.allowed_left_angle < h.allowed_right_angle)
	{
		r = h.allowed_right_angle;
		if(DEBUG)
			ROS_INFO("9: Allowed right angle chosen");
	}

	// converts the turn angle to radians
	r *= 2*M_PI / LRS_size;
	if (r > M_PI)
		r -= 2*M_PI;

	return r;
}

void LidarNavigator::publishVisualization(const hole& h, double turn_angle)
{
	vizMarker.markers.clear();

	if (h.left_angle != 0 && h.right_angle != 0)
	{
		visualization_msgs::Marker marker;

		marker.header.frame_id = "neato_lidar";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = cos(-turn_angle/2);
		marker.pose.orientation.y = sin(-turn_angle/2);
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 0.0;
		marker.scale.x = 0.4;
		marker.scale.y = 0.6;
		marker.scale.z = 0.5;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		vizMarker.markers.push_back(marker);
	}

	marker_pub.publish(vizMarker);
}
