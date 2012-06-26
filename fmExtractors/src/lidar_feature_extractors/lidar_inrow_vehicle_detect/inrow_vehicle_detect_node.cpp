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

#include "ros/ros.h"

#include "inrow_vehicle_detector.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "inrow_vehicle_detect_node");

  ros::NodeHandle nh("~");
  ros::NodeHandle n;

  std::string lidar_sub_topic;
  std::string viz_marker_pub_topic;
  std::string row_pub_topic;
  std::string point_cloud_pub_topic;
  std::string point_cloud_rotated_pub_topic;
  std::string position_sub_topic;
  std::string vehicle_position_pub_topic;
  std::string map_pub_topic;
  std::string warhorse_state_topic;
  std::string nav_spec_sub_topic;


  int numberOfParticles;
  double len_x;
  double len_y;
  double max_ang;
  double measurements_noise;
  double movement_noise;
  double turning_noise;

  double map_resolution;

  nh.param<int>("particles", numberOfParticles, 1000);
  nh.param<double>("length_x", len_x, 0.5);
  nh.param<double>("length_y", len_y, 0.5);
  nh.param<double>("max_ang", max_ang, M_PI/4);
  nh.param<double>("measurement_noise", measurements_noise, 0.05);
  nh.param<double>("movement_noise", movement_noise, 0.01);
  nh.param<double>("turning_noise", turning_noise, M_PI/16);

  nh.param<double>("map_resolution",map_resolution,0.025);

  nh.param<std::string>("lidar_sub_topic", lidar_sub_topic, "/fmSensors/lidar_scan_data");
  nh.param<std::string>("visualization_marker_pub_topic", viz_marker_pub_topic, "/fmExtractors/viz_marker_particle");
  nh.param<std::string>("point_cloud_pub_topic", point_cloud_pub_topic, "/fmExtractors/point_cloud");
  nh.param<std::string>("position_sub_topic", position_sub_topic, "/fmExtractors/vehicle_coordinate");
  nh.param<std::string>("vehicle_position_pub_topic", vehicle_position_pub_topic, "/fmExtractors/vehicle_position");
  nh.param<std::string>("map_pub_topic", map_pub_topic, "/fmExtractors/map");
  nh.param<std::string> ("warhorse_state_topic", warhorse_state_topic, "/state"); //Specify the publisher name
  nh.param<std::string>("nav_spec_sub_topic", nav_spec_sub_topic, "/fmDecisionMakers/nav_spec");


  InRowVehicleDetector rd(numberOfParticles, len_x, len_y, max_ang, measurements_noise, movement_noise, turning_noise, map_resolution);

  rd.laser_scan_sub = nh.subscribe<sensor_msgs::LaserScan> (lidar_sub_topic.c_str(), 2, &InRowVehicleDetector::processLaserScan, &rd);

  rd.position_sub = nh.subscribe<fmMsgs::vehicle_coordinate> (position_sub_topic.c_str(), 2, &InRowVehicleDetector::positionCallback, &rd);

  rd.marker_pub = n.advertise<visualization_msgs::MarkerArray>(viz_marker_pub_topic.c_str(), 1);
  rd.point_cloud_pub = n.advertise<sensor_msgs::PointCloud>(point_cloud_pub_topic.c_str(), 1);
  rd.vehicle_position_pub = n.advertise<fmMsgs::vehicle_position>(vehicle_position_pub_topic.c_str(), 1);
  rd.map_pub = n.advertise<nav_msgs::OccupancyGrid>(map_pub_topic.c_str(),1);
  rd.warhorse_state_sub = n.subscribe<fmMsgs::warhorse_state>(warhorse_state_topic.c_str(),1,&InRowVehicleDetector::stateHandler,&rd);
  rd.nav_spec_sub = n.subscribe<fmMsgs::navigation_specifications>(nav_spec_sub_topic.c_str(),1,&InRowVehicleDetector::navSpecHandler,&rd);

  ros::spin();

  return 0;
}
