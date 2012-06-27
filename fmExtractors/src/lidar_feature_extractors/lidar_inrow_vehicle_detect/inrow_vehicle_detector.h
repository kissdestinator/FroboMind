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
#include "fmMsgs/warhorse_state.h"
#include "fmMsgs/navigation_specifications.h"
#include "fmMsgs/blocked_row.h"

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

  fmMsgs::vehicle_position vehicle_position;

  int numberOfParticles;
  double len_x,off_x,len_y,off_y,max_ang, measurements_noise, movement_noise, turning_noise;

  fmMsgs::vehicle_coordinate position;
  fmMsgs::vehicle_coordinate last_position;
  fmMsgs::vehicle_coordinate delta_position;
  fmMsgs::vehicle_coordinate first_position;

  tf::TransformBroadcaster map_broadcaster;

  void detectBlockedRow(const sensor_msgs::PointCloud& pointCloud, const fmMsgs::vehicle_position& position);

public:

  ros::Publisher point_cloud_pub;

  ros::Publisher marker_pub;

  ros::Publisher vehicle_position_pub;

  ros::Publisher map_pub;

  ros::Subscriber laser_scan_sub;

  ros::Subscriber position_sub;

  ros::Subscriber warhorse_state_sub;

  ros::Subscriber nav_spec_sub;

  ros::Publisher blocked_row_pub;

  laser_geometry::LaserProjection projector;

  nav_msgs::OccupancyGrid map;

  fmMsgs::warhorse_state warhorse_state;

  InRowVehicleDetector(int NumberOfParticles,double Len_x,double Len_y,double Max_ang, double Measurements_noise, double Movement_noise, double Turning_noise, double map_res);
  void processLaserScan(sensor_msgs::LaserScan laser_scan);
  void positionCallback(const fmMsgs::vehicle_coordinate::ConstPtr& pos);
  void stateHandler(const fmMsgs::warhorse_stateConstPtr& msg);
  void navSpecHandler(const fmMsgs::navigation_specificationsConstPtr& msg);
  void buildMap();
  void publishMap();
  void createMap();
  void sendMapTransform(fmMsgs::vehicle_position vp);


};

#endif /* INROW_VEHICLE_DETECTOR_H_ */
