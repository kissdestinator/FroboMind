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

#define SAFETY_RANGE 0.20
#define NAV_RANGE 0.6
#define MIN_RANGE 0.06
#define MIN_WIDTH 0.35
#define GOAL 0.0

#define K_TURN 4.0

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
	ROS_INFO("CALLBACK - LIDAR NAV");

	std::vector<hole> holes;

	for (int i = 0; i < laser_scan->get_ranges_size(); i++)
	{
		
		if ((laser_scan->ranges[i] < SAFETY_RANGE && laser_scan->ranges[i] > MIN_RANGE) && (i > 315 || i < 45))
		{
			geometry_msgs::TwistStamped twist;
			twist.twist.linear.x = 0;
			twist.twist.angular.z = 0;
			ROS_INFO("Objekt i vejen..!");
			velocity_pub.publish(twist);
			return;
		}		
		else if (laser_scan->ranges[i] > NAV_RANGE || laser_scan->ranges[i] < MIN_RANGE)
		{
			int done = 0;
			int j = i;
			hole h;
			while(laser_scan->ranges[j] > NAV_RANGE || laser_scan->ranges[j] < MIN_RANGE || done > 2)
			{
				j = (j+1);
				if (j >=360 )
					done++;
				j %= 360;
			}
			h.right_angle = j;
			j = i;
			while(laser_scan->ranges[j] > NAV_RANGE || laser_scan->ranges[j] < MIN_RANGE)
				j = (j-1) % laser_scan->get_ranges_size();
			h.left_angle = j;

			if (!done)
				i = h.right_angle;
			else
				i = laser_scan->get_ranges_size();

			if (h.left_angle > h.right_angle)
				h.angle = 360-h.left_angle + h.right_angle;
			else
				h.angle = h.right_angle - h.left_angle;

			h.width = MIN(laser_scan->ranges[h.right_angle],laser_scan->ranges[h.left_angle]) * sin((h.angle/2)*M_PI/180) * 2.0;
			h.center_angle = (h.left_angle + h.angle/2.0);
			if (h.center_angle >= 360)
				h.center_angle -= 360;
			if (h.width > MIN_WIDTH)
			{
				holes.push_back(h);
				ROS_INFO("Right: %d, Left: %d, Center: %f, width: %f, angle: %f",h.right_angle, h.left_angle, h.center_angle, h.width, h.angle);
			}
		}
	}
	if (holes.size() > 0)
	{
		hole h = holes[0];
		for (int i = 0; i < holes.size(); i++)
		{
			if (MIN(abs(holes[i].center_angle - GOAL),abs(holes[i].center_angle - 360 - GOAL)) < abs(h.center_angle - GOAL))
				h = holes[i];
			if (MIN(abs(holes[i].center_angle - GOAL),abs(holes[i].center_angle + 360 - GOAL)) < abs(h.center_angle - GOAL))
				h = holes[i];
		}

		h.allowed_left_angle = h.left_angle + ( MIN_WIDTH/2 * h.angle/h.width );
		h.allowed_right_angle = h.right_angle - ( MIN_WIDTH/2 * h.angle/h.width );

		if ((GOAL < h.allowed_right_angle && GOAL > h.allowed_left_angle && h.left_angle < h.right_angle) ||
			(GOAL < h.allowed_right_angle && h.left_angle > h.right_angle) ||
			(GOAL > h.allowed_left_angle && h.left_angle > h.right_angle))
			turn_angle = GOAL;
		else if (abs(h.allowed_left_angle - GOAL) < abs(h.allowed_right_angle - GOAL) && h.left_angle > h.right_angle)
			turn_angle = h.allowed_left_angle;
		else if (abs(h.allowed_right_angle - GOAL) < abs(h.allowed_left_angle - GOAL) && h.left_angle > h.right_angle)
			turn_angle = h.allowed_right_angle;
		else if (abs(GOAL-360-h.allowed_left_angle) < abs(GOAL-h.allowed_right_angle) && h.left_angle < h.right_angle)
			turn_angle = h.allowed_left_angle;
		else if (abs(GOAL+360-h.allowed_right_angle) < abs(GOAL-h.allowed_left_angle) && h.left_angle < h.right_angle)
			turn_angle = h.allowed_right_angle;
		else if (abs(GOAL-h.allowed_left_angle) < abs(GOAL-h.allowed_right_angle) && h.left_angle < h.right_angle)
			turn_angle = h.allowed_left_angle;
		else if (abs(GOAL-h.allowed_right_angle) < abs(GOAL-h.allowed_left_angle) && h.left_angle < h.right_angle)
			turn_angle = h.allowed_right_angle;

		turn_angle *= M_PI / 180;

		geometry_msgs::TwistStamped twist;
		twist.twist.linear.x = 0.5;
		twist.twist.angular.z = -turn_angle * K_TURN;
		ROS_INFO("ARA: %f, ALA: %f, CA: %f, TA: %f",h.allowed_right_angle, h.allowed_left_angle, h.center_angle, twist.twist.angular.z);
		velocity_pub.publish(twist);
	}

}

