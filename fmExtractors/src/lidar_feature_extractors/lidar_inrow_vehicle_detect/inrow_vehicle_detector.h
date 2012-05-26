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

  double map_size_x;
  double map_size_y;
  double map_resolution;
  double row_width;
  double row_length;
  double row_spacing;
  double no_of_rows;
  double start_x;
  double start_y;

  fmMsgs::vehicle_coordinate position;
  fmMsgs::vehicle_coordinate last_position;
  fmMsgs::vehicle_coordinate delta_position;
  fmMsgs::vehicle_coordinate first_position;

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

  InRowVehicleDetector(int numberOfParticles,double len_x,double off_x,double len_y,double off_y,double max_ang, double measurements_noise, double movement_noise, double turning_noise);
  void processLaserScan(sensor_msgs::LaserScan laser_scan);
  void positionCallback(const fmMsgs::vehicle_coordinate::ConstPtr& pos);
  nav_msgs::OccupancyGrid buildMap();
  nav_msgs::OccupancyGrid buildHollowMap();
  void publishMap();
  void createMap(double MAP_SIZE_X, double MAP_SIZE_Y, double MAP_RESOLUTION, double ROW_WIDTH, double ROW_LENGTH, double ROW_SPACING, double NO_OF_ROWS, double START_X,double START_Y);
  void sendMapTransform(fmMsgs::vehicle_position vp);

};

#endif /* INROW_VEHICLE_DETECTOR_H_ */
