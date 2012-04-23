/****************************************************************************
# Lidar row detect node
# Copyright (c) 2011 Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
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
# File: AES25_node.cpp
# Purpose: AES25 actuator driver.
# Project: Field Robot - Vehicle Interface Computer
# Author: Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
# Created: Jun 3, 2011 Søren Hundevadt Nielsen, Source written
****************************************************************************/

#include "lidar_navigator.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_navigator");

  LidarNavigator ln;

  ros::NodeHandle nh("~");
  ros::NodeHandle n;

  std::string lidar_sub_topic;
  std::string viz_marker_pub_topic;
  std::string velocity_pub_topic;
  std::string position_sub_topic;

  nh.param<std::string>("lidar_sub_topic", lidar_sub_topic, "/fmSensors/lidar_scan_data");
  nh.param<std::string>("visualization_marker_pub_topic", viz_marker_pub_topic, "/fmExtractors/viz_marker_navigator");
  nh.param<std::string>("velocity_pub_topic", velocity_pub_topic, "/speed_from_joystick");
  nh.param<std::string>("vehicle_coordinate", position_sub_topic, "/fmExtractors/vehicle_coordinate");


  ln.laser_scan_sub = nh.subscribe<sensor_msgs::LaserScan> (lidar_sub_topic.c_str(), 2, &LidarNavigator::processLaserScan, &ln);
  ln.position_sub = nh.subscribe<fmMsgs::vehicle_coordinate> (position_sub_topic.c_str(), 2, &LidarNavigator::positionCallback, &ln);

  ln.velocity_pub = n.advertise<geometry_msgs::TwistStamped>(velocity_pub_topic.c_str(), 1);
  ln.marker_pub = n.advertise<visualization_msgs::MarkerArray>(viz_marker_pub_topic.c_str(), 1);

  ros::spin();

  return 0;
}
