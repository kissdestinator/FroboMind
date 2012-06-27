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

#include "inrow_vehicle_detector.h"

InRowVehicleDetector::InRowVehicleDetector(int NumberOfParticles,double Len_x,double Len_y,double Max_ang, double Measurements_noise, double Movement_noise, double Turning_noise, double map_res)
{
	numberOfParticles = NumberOfParticles;
	len_x = Len_x;
	len_y = Len_y;
	max_ang = Max_ang;
	measurements_noise = Measurements_noise;
	movement_noise = Movement_noise;
	turning_noise = Turning_noise;

	map_resolution = map_res;

	first_position.x = 0;
	first_position.y = 0;
	first_position.th = 0;
	last_position.x = 0;
	last_position.y = 0;
	last_position.th = 0;

	off_x = 10;
	off_y = 10;

	vehicle_position.position.x = 0;
	vehicle_position.position.y = 0;
	vehicle_position.position.th = 0;

	particlefilter = ParticleFilter(numberOfParticles,len_x,off_x,len_y,off_y,max_ang, measurements_noise, movement_noise, turning_noise);

}

void InRowVehicleDetector::createMap()
{
	buildMap();
}


void InRowVehicleDetector::positionCallback(const fmMsgs::vehicle_coordinate::ConstPtr& pos)
{
	if (last_position.x == 0 && last_position.y == 0 && last_position.th == 0)
	{
		last_position.x = pos->x;
		last_position.y = pos->y;
		last_position.th = pos->th;
	}
	position.x = pos->x;
	position.y = pos->y;
	position.th = pos->th;
}

void InRowVehicleDetector::navSpecHandler(const fmMsgs::navigation_specificationsConstPtr& msg)
{
	off_x = msg->start_x;
	off_y = msg->start_y;
	start_x = msg->row_offset_x;
	start_y = msg->row_offset_y;
	no_of_rows = msg->no_of_rows;
	row_length = msg->row_length;
	row_spacing = msg->row_spacing;
	row_width = msg->row_width;
	map_size_x = (double)(int)(2 * start_x + (no_of_rows * (row_width + row_spacing)));
	map_size_y = (double)(int)(2 * start_y + row_length);
	if (map_size_x > map_size_y)
		map_size_y = map_size_x;
	if (map_size_y > map_size_x)
		map_size_x = map_size_y;

}

void InRowVehicleDetector::stateHandler(const fmMsgs::warhorse_stateConstPtr& msg)
{
	if (msg->drive_state == warhorse_state.DRIVE && warhorse_state.drive_state == warhorse_state.STOP)
	{
		createMap();
		ROS_INFO("Start_x, %f start_y %f", off_x, off_y);
		particlefilter.resetParticleFilter(off_x,off_y);
	}
	warhorse_state.drive_state = msg->drive_state;
	warhorse_state.task_state = msg->task_state;
}

fmMsgs::vehicle_coordinate calcPositionChange(fmMsgs::vehicle_coordinate position, fmMsgs::vehicle_coordinate last_position)
{
	fmMsgs::vehicle_coordinate r;

	r.x = position.x - last_position.x;
	r.y = position.y - last_position.y;
	r.th = position.th - last_position.th;
	if (r.th > 2*M_PI)
		r.th -= 2*M_PI;
	if (r.th < 0)
		r.th += 2*M_PI;

	return r;
}

void InRowVehicleDetector::sendMapTransform(fmMsgs::vehicle_position vp)
{
	geometry_msgs::Quaternion map_quat = tf::createQuaternionMsgFromYaw(-vp.position.th);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped map_trans;
	map_trans.header.stamp = ros::Time::now();
	map_trans.header.frame_id = "/map";
	map_trans.child_frame_id = "/vehicle";

	map_trans.transform.translation.x = vp.position.y;
	map_trans.transform.translation.y = vp.position.x;
	map_trans.transform.translation.z = 0.0;
	map_trans.transform.rotation = map_quat;

	//send the transform
	map_broadcaster.sendTransform(map_trans);
}

void InRowVehicleDetector::detectBlockedRow(const sensor_msgs::PointCloud& pointCloud, const fmMsgs::vehicle_position& position)
{
	double x = vehicle_position.position.x;
	double y = vehicle_position.position.y;
	double th = vehicle_position.position.th;

	int hits = 0;
	int row = 1;
	bool row_found = false;
	double row_left_border;
	double row_right_border;

	double y_lower;
	double y_upper;
	double x_lower;
	double x_upper;

	while(!row_found && row != 20)
	{
		row_left_border = start_x + (row * row_width) + ((row - 1) * row_spacing);
		row_right_border = row_left_border + row_spacing;
		if (x > row_left_border && x < row_right_border)
			row_found = true;
		else
			row++;
	}

	// robotten peger opad
	if (vehicle_position.position.th > 1.75*M_PI || vehicle_position.position.th < 0.25*M_PI)
	{
		if (y > start_y && y < start_y + row_length - 1)
		{
			y_lower = y + 0.25;
			y_upper = y + 1;
			x_lower = row_left_border + 0.15;
			x_upper = row_right_border - 0.15;
			ROS_INFO("OPAD, y_lower: %f, y_upper: %f, x_lower: %f, x_upper: %f", y_lower, y_upper, x_lower, x_upper);
			for (int j = 0; j < pointCloud.points.size(); j++)
			{
				// transponer laser målingerne ind omkring (0,0) for at muliggøre sortering
				geometry_msgs::Point32 t;
				t.x = x + pointCloud.points[j].y * cos(th) - pointCloud.points[j].x * sin(th);
				t.y = y + pointCloud.points[j].y * sin(th) + pointCloud.points[j].x * cos(th);

				// Beregn fejl hvis målingen er gyldig
				if ((t.x > x_lower && t.x < x_upper) && (t.y > y_lower && t.y < y_upper))
				{
					hits++;
					ROS_INFO("point_x: %f, point_y: %f, scan_point_x: %f, scan_point_y: %f",t.x,t.y,pointCloud.points[j].x,pointCloud.points[j].y);
				}
			}
		}
	}
	// Robotten peger nedad
	else if (vehicle_position.position.th > 0.75*M_PI && vehicle_position.position.th < 1.25*M_PI)
	{
		if (y > start_y + 1 && y < start_y + row_length)
		{
			y_lower = y - 1;
			y_upper = y - 0.25;
			x_lower = row_left_border + 0.15;
			x_upper = row_right_border - 0.15;
			ROS_INFO("NEDAD, y_lower: %f, y_upper: %f, x_lower: %f, x_upper: %f", y_lower, y_upper, x_lower, x_upper);
			for (int j = 0; j < pointCloud.points.size(); j++)
			{
				// transponer laser målingerne ind omkring (0,0) for at muliggøre sortering
				geometry_msgs::Point32 t;
				t.x = x + pointCloud.points[j].y * cos(th) - pointCloud.points[j].x * sin(th);
				t.y = y + pointCloud.points[j].y * sin(th) + pointCloud.points[j].x * cos(th);

				// Beregn fejl hvis målingen er gyldig
				if ((t.x > x_lower && t.x < x_upper) && (t.y > y_lower && t.y < y_upper))
				{
					hits++;
					ROS_INFO("point_x: %f, point_y: %f, scan_point_x: %f, scan_point_y: %f",t.x,t.y,pointCloud.points[j].x,pointCloud.points[j].y);
				}
			}
		}
	}
	if (hits > 10)
	{
		fmMsgs::blocked_row br;
		br.blocked = true;
		br.position.x = (x_lower + x_upper) / 2.0;
		br.position.y = (y_lower + y_upper) / 2.0;

		for(int y = br.position.y/map_resolution; y < br.position.y/map_resolution + 3; y++)
		{
			for(int x = (x_lower-0.15)/map_resolution; x <= (x_upper+0.15)/map_resolution; x++)
			{
				map.data[map.info.width*x+y] = 100;
			}
		}
		blocked_row_pub.publish(br);
	}
}

void InRowVehicleDetector::processLaserScan(sensor_msgs::LaserScan laser_scan)
{
	sensor_msgs::PointCloud cloud;

	laser_scan.angle_increment = -laser_scan.angle_increment;
	double temp = laser_scan.angle_max;
	laser_scan.angle_max = laser_scan.angle_min;
	laser_scan.angle_min = temp;

	try
    {
    	projector.projectLaser(laser_scan, cloud);
    }
    catch (tf::TransformException& e)
    {
        std::cout << "Error: " << e.what();
        return;
    }

    cloud.header.frame_id = "/vehicle";
    cloud.header.stamp = ros::Time::now();
    point_cloud_pub.publish(cloud);


	delta_position = calcPositionChange(position,last_position);

	last_position = position;

    if (warhorse_state.drive_state == warhorse_state.DRIVE && warhorse_state.task_state != warhorse_state.MANUAL_DRIVE)
    {
    	vehicle_position = particlefilter.update(cloud,delta_position,map);

        ROS_INFO("Position in map: x: %.3f y: %.3f th: %.3f",vehicle_position.position.x ,vehicle_position.position.y,vehicle_position.position.th);
    	//ROS_INFO("Odom: x: %.3f y: %.3f th: %.3f",position.x ,position.y,position.th);

    	particlefilter.updateParticlesMarker();

    	visualization_msgs::MarkerArray markerArray = particlefilter.getParticlesMarker();

    	marker_pub.publish(markerArray);
    	if (warhorse_state.task_state == warhorse_state.TASK2)
    		detectBlockedRow(cloud,vehicle_position);
    }

	publishMap();
	sendMapTransform(vehicle_position);

	vehicle_position_pub.publish(vehicle_position);
}

void InRowVehicleDetector::publishMap()
{
	map_pub.publish(map);
}

void InRowVehicleDetector::buildMap()
{

	map.info.height = (uint32_t)(map_size_y / map_resolution);
	map.info.width = (uint32_t)(map_size_x / map_resolution);
	map.info.resolution = map_resolution;
	map.info.map_load_time = ros::Time::now();
	map.header.frame_id = "/map";
	map.header.stamp = ros::Time::now();

	int range_x = map.info.width;
	int range_y = map.info.height;

	map.data.clear();

	for (int y = 0; y < range_y; y++)
		for (int x = 0; x < range_x; x++)
			map.data.push_back(0);

	int smooth = 0.05 / map_resolution;

	for (double i = 0; i < no_of_rows; i++)
	{
		int offset_x = (start_x + i*(row_spacing+row_width)) / map_resolution;
		int offset_y = start_y / map_resolution;
		int end_x = offset_x+(row_width/map_resolution);
		int end_y = offset_y+(row_length/map_resolution);
		ROS_INFO("offset_x: %d, offset_y: %d",offset_x,offset_y);

		for (int x = offset_x; x <= end_x; x++)
		{
			for (int y = offset_y; y <= end_y; y++)
			{
				bool updated = false;
				if (y == offset_y)
				{
					map.data[range_y*x+y] = 100;
					updated = true;
				}
				if (y == end_y)
				{
					map.data[range_y*x+y] = 100;
					updated = true;
				}
				if (x == offset_x)
				{
					map.data[range_y*x+y] = 100;
					updated = true;
				}
				if (x == end_x)
				{
					map.data[range_y*x+y] = 100;
					updated = true;
				}
				if (updated)
				{
					for (int x1 = x - smooth; x1 <= x + smooth; x1++)
					{
						for (int y1 = y - smooth; y1 <= y + smooth; y1++)
						{
							int update_value;
							if (x - x1 == 0 && y - y1 == 0)
								;
							else if (x - x1 == 0)
								update_value = 100 / abs(y - y1);
							else if (y - y1 == 0)
								update_value = 100 / abs(x - x1);
							else
								update_value = 100 / sqrt(pow(y - y1,2) + pow(x - x1,2));
							if (update_value > map.data[range_y*x1+y1])
								map.data[range_y*x1+y1] = update_value;
						}
					}
				}
			}
		}
	}
}
