#ifndef INROW_VEHICLE_DETECTOR_H_
#define INROW_VEHICLE_DETECTOR_H_


#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"
#include "fmMsgs/Vector3.h"
#include "fmMsgs/vehicle_position.h"
#include "fmMsgs/vehicle_coordinate.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"

#include "particle_filter.h"

#include <tf/transform_broadcaster.h>

class InRowVehicleDetector
{
private:

  ParticleFilter particlefilter;

  double    right_distance_;
  double    left_distance_;
  double    right_angle_;
  double	left_angle_;
  double 	e_angle;
  double 	e_distance;

  fmMsgs::vehicle_coordinate position;
  fmMsgs::vehicle_coordinate last_position;
  fmMsgs::vehicle_coordinate delta_position;

  tf::TransformBroadcaster map_broadcaster;

public:

  ros::Publisher point_cloud_pub;

  ros::Publisher marker_pub;

  ros::Publisher vehicle_position_pub;

  ros::Publisher map_pub;

  ros::Subscriber laser_scan_sub;

  ros::Subscriber position_sub;

  laser_geometry::LaserProjection projector;

  nav_msgs::OccupancyGrid map;

  InRowVehicleDetector();
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
  void positionCallback(const fmMsgs::vehicle_coordinate::ConstPtr& pos);
  nav_msgs::OccupancyGrid buildMap();
  nav_msgs::OccupancyGrid buildHollowMap();
  void publishMap();

};

#endif /* INROW_VEHICLE_DETECTOR_H_ */
