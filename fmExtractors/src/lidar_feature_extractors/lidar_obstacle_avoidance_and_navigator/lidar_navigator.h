#ifndef LIDAR_NAVIGATOR_H_
#define LIDAR_NAVIGATOR_H_

#include "ros/ros.h"

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include "geometry_msgs/TwistStamped.h"
#include "fmMsgs/path.h"
#include "fmMsgs/nav_points.h"
#include "fmMsgs/Vector3.h"
#include "fmMsgs/vehicle_coordinate.h"
#include "nr3.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

struct hole
{
	hole()
	{
		left_angle = 0; right_angle = 0; width = 0;
	}
	hole(double left_ang, double right_ang, double wid)
	{
		left_angle = left_ang; right_angle = right_ang; width = wid;
	}

	int left_angle;
	int right_angle;
	double center_angle;
	double width;
	double angle;

	double allowed_left_angle;
	double allowed_right_angle;
};

class LidarNavigator
{
private:
	double x,y,th;
	double old_x,old_y,old_th;

	double velocity;
	double turn_angle;

public:

  ros::Publisher marker_pub;
  ros::Publisher velocity_pub;

  ros::Subscriber laser_scan_sub;
  ros::Subscriber position_sub;

  LidarNavigator();
  void processLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan );
  void positionCallback(const fmMsgs::vehicle_coordinateConstPtr& position);

};

#endif /* INROW_VEHICLE_DETECTOR_H_ */
