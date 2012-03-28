#ifndef INROW_VEHICLE_DETECTOR_H_
#define INROW_VEHICLE_DETECTOR_H_


#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "laser_geometry/laser_geometry.h"
//#include "sensor_msgs/Image.h"
//#include "pcl/io/pcd_io.h"
#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"
#include "fmMsgs/row.h"
//#include "tf/transform_listener.h"
#include "fmMsgs/Vector3.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "particle_filter.h"

class InRowVehicleDetector
{
private:

  fmMsgs::row rows_;

  ParticleFilter particlefilter;

  double    right_distance_;
  double    left_distance_;
  double    right_angle_;
  double	left_angle_;
  double 	e_angle;
  double 	e_distance;

  double old_x;
  double old_y;
  double old_theta;
  double x;
  double y;
  double theta;
  double dx;
  double dy;
  double dtheta;

public:

  ros::Publisher point_cloud_pub;
  ros::Publisher point_cloud_rotated_pub;

  ros::Publisher marker_pub;

  ros::Publisher row_pub;

  ros::Subscriber laser_scan_sub;
  ros::Subscriber position_sub;

  laser_geometry::LaserProjection projector;

  InRowVehicleDetector();
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
  void positionCallback(const fmMsgs::Vector3::ConstPtr& position);

};

#endif /* INROW_VEHICLE_DETECTOR_H_ */
