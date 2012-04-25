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

int mMod(int number, int modulus)
{
     int result = number % modulus;

     if (result < 0) result += modulus;

     return result;
}

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

	double temp_heading = desired_heading;
	if (desired_heading > M_PI)
		temp_heading -= 2 * M_PI;
	temp_heading = (int)(temp_heading * LRS_size / 2 * M_PI);

	int hole_found = 0;

	for (double test_range = nav_range; test_range >= safety_range && !hole_found; test_range -= (nav_range - safety_range)/3)
	{
		int index = temp_heading;

		double mca = (asin((min_clearance_width/2.0)/test_range) * 2.0 * LRS_size / (2 * M_PI));
		int minimum_clearance_angle = (int)mca;
		int step_size = 15;//(int)((double)minimum_clearance_angle/2.0);
		int half_min_clerance_angle = minimum_clearance_angle/2;
		int step = step_size;

		ROS_INFO("MCA: %f, intMCA %d, halvIntMCA %d",mca,minimum_clearance_angle,half_min_clerance_angle);

		for (int i = 0; i < (LRS_size/step_size); i++)
			{
				int center(0), left_line(0), right_line(0);

				while((ranges[mMod(left_line+index,LRS_size)] > test_range || ranges[mMod(left_line+index,LRS_size)] < min_range) && abs(left_line) < (int)LRS_size/4)
					left_line--;

				while((ranges[mMod(right_line+index,LRS_size)] > test_range || ranges[mMod(right_line+index,LRS_size)] < min_range) && abs(right_line) < (int)LRS_size/4)
					right_line++;

				if (right_line-left_line > minimum_clearance_angle)
				{
					if (temp_heading > index+left_line+half_min_clerance_angle && temp_heading < index+right_line-half_min_clerance_angle)
						center = temp_heading;
					else if (temp_heading-index < left_line+half_min_clerance_angle)
						center = index+left_line+half_min_clerance_angle;
					else if (temp_heading-index > right_line-half_min_clerance_angle)
						center = index+right_line-half_min_clerance_angle;

					turn_angle = center * 2*M_PI/LRS_size;

					// Calculate and publish the vehicle speed
					double max_vel = max_velocity * safetyCheck(ranges) * test_range;
					ROS_INFO("index: %d, left: %d, center: %d, right: %d, turn angle: %.3f, test range: %.3f", index, left_line, center, right_line,turn_angle,test_range);
					calcAndPublishSpeed(turn_angle,max_vel);

					i = (LRS_size/step_size);
					hole_found = 1;
				}
				else
				{
					if (mMod(i,2))
						index += step;
					else
						index -= step;
					step += step_size;
				}
			}
	}

	if (!hole_found)
	{
		ROS_INFO("No holes wide enough found..! - setting speed to zero");
		calcAndPublishSpeed(0,0);
	}

}

double LidarNavigator::safetyCheck(const std::vector<double>& ranges)
{
	int half_min_clearance_angle = (min_clearance_angle / 360 * ranges.size()) / 2;
	for (int i = -half_min_clearance_angle; i <half_min_clearance_angle; i++)
		if (ranges[mMod(i,ranges.size())] < safety_range && ranges[mMod(i,ranges.size())] > min_range)
			{
				ROS_INFO("Obstacle in the way.! angular velocity set to zero.!");
				return 0;
			}
	return 1;
}

void LidarNavigator::calcAndPublishSpeed(double turn_angle, double velocity)
{
	double vel = 0;
	double ang_vel = 0;

	// Calculate velocity
	if (abs(turn_angle) < 15 * DEG2RAD)
		vel = velocity;
	else if (abs(turn_angle) < 90 * DEG2RAD)
		vel = velocity * (90 - (abs(turn_angle)*RAD2DEG)) / 75;

	// Calculate Angular velocity
	ang_vel = turn_angle * K_ang_vel;

	publishVisualization(turn_angle);

	geometry_msgs::TwistStamped twist;
	twist.twist.linear.x = vel;
	twist.twist.angular.z = ang_vel;
	velocity_pub.publish(twist);

	ROS_INFO("Turn Angle: %.3f, Velocity: %.3f, Angular Velocity: %.3f",turn_angle,vel,ang_vel);
}

void LidarNavigator::publishVisualization(double turn_angle)
{
	vizMarker.markers.clear();

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

	marker_pub.publish(vizMarker);
}
