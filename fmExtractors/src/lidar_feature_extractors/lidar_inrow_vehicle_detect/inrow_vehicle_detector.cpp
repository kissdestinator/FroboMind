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

#define MAP_SIZE_X 40
#define MAP_SIZE_Y 40
#define MAP_RESOLUTION 0.05

#define ROW_WIDTH 0.40
#define ROW_LENGTH 20
#define ROW_SPACING 0.75
#define NO_OF_ROWS 20
#define START_X 10
#define START_Y 10

InRowVehicleDetector::InRowVehicleDetector()
{
	particlefilter = ParticleFilter(1000,0,0,0.75,0.375,M_PI/2,0,0.1,0.1);

	old_theta = 0;
	old_x = 0;
	old_y = 0;

	map = buildMap();
}

void InRowVehicleDetector::positionCallback(const fmMsgs::Vector3::ConstPtr& position)
{
	x = position->x;
	y = position->y;
	theta = position->th;
}

void InRowVehicleDetector::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_scan )
{
	sensor_msgs::PointCloud cloud;

	try
    {
    	projector.projectLaser(*laser_scan, cloud);
    }
    catch (tf::TransformException& e)
    {
        std::cout << "Error: " << e.what();
        return;
    }

    cloud.header.frame_id = "base_link";
    point_cloud_pub.publish(cloud);

	fmMsgs::vehicle_position vp = particlefilter.update(cloud,dx,dy,dtheta);

	vehicle_position_pub.publish(vp);

    particlefilter.updateParticlesMarker();

	visualization_msgs::MarkerArray markerArray = particlefilter.getParticlesMarker();

	marker_pub.publish(markerArray);
}

void InRowVehicleDetector::publishMap()
{
	map_pub.publish(map);
}

nav_msgs::OccupancyGrid InRowVehicleDetector::buildMap()
{
	nav_msgs::OccupancyGrid r;
	nav_msgs::MapMetaData temp;

	temp.height = MAP_SIZE_Y / MAP_RESOLUTION;
	temp.width = MAP_SIZE_X / MAP_RESOLUTION;
	temp.resolution = MAP_RESOLUTION;
	temp.map_load_time = ros::Time::now();

	r.info = temp;
	r.header.frame_id = "/map";
	r.header.stamp = ros::Time::now();

	int range_x = r.info.width;
	int range_y = r.info.height;

	for (int y = 0; y < range_y; y++)
	{
		for (int x = 0; x < range_x; x++)
		{
			r.data.push_back(0);
		}
	}

	for (int i = 0; i < NO_OF_ROWS; i++)
	{
		int offset_x = (START_X + i*(ROW_SPACING+ROW_WIDTH)) / MAP_RESOLUTION;
		int offset_y = START_Y / MAP_RESOLUTION;

		for (int y = offset_y; y < offset_y+(ROW_LENGTH/MAP_RESOLUTION); y++)
		{
			for (int x = offset_x; x < offset_x+(ROW_WIDTH/MAP_RESOLUTION); x++)
			{
				r.data[range_x*y+x] = 100;
			}
		}
	}

	return r;
}

