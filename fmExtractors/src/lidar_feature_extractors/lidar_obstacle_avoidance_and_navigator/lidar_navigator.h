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

#include "pid_regulator.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class LidarNavigator
{
private:
	double x,y,th;
	double old_x,old_y,old_th;

	double velocity;
	double turn_angle;

	double max_angular_velocity;
	double max_velocity;

	double current_velocity;

	double nav_range;
	double safety_range;
	double min_range;
	double min_clearance_width;
	double min_clearance_angle;
	double desired_heading;

	double P_ang_vel;
	double I_ang_vel;
	double D_ang_vel;

	PIDRegulator pid_ang_vel;

	int laser_inverted;

	visualization_msgs::MarkerArray vizMarker;

	double calcTurnAngle(double goal, int LRS_size);
	void publishVisualization(double turn_angle);
	void calcAndPublishSpeed(double turn_angle, double velocity);
	double safetyCheck(const std::vector<double>& ranges);

public:

  ros::Publisher marker_pub;
  ros::Publisher velocity_pub;

  ros::Subscriber laser_scan_sub;
  ros::Subscriber position_sub;

  LidarNavigator();
  void processLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan );
  void positionCallback(const fmMsgs::vehicle_coordinateConstPtr& position);

  void setNavRange(double range) 				{ nav_range = range; }
  void setSafetyRange(double range) 			{ safety_range = range; }
  void setMinRange(double range) 				{ min_range = range; }
  void setLaserInverted(int inverted) 			{ laser_inverted = inverted; }
  void setMaxAllowedAngularVelocity(double av) 	{ max_angular_velocity = av; }
  void setMaxAllowedVelocity(double v) 			{ max_velocity = v; }
  void setDesiredHeading(double heading)		{ desired_heading = heading; }
  void setMinClearanceAngle(double angle)		{ min_clearance_angle = angle; }
  void setMinClearance(double clearance)		{ min_clearance_width = clearance; }
  void setPAngularVelocity(double ang_vel)		{ P_ang_vel = ang_vel; pid_ang_vel.setP(ang_vel);}
  void setIAngularVelocity(double ang_vel)		{ I_ang_vel = ang_vel; pid_ang_vel.setI(ang_vel);}
  void setDAngularVelocity(double ang_vel)		{ D_ang_vel = ang_vel; pid_ang_vel.setD(ang_vel);}

};

#endif /* INROW_VEHICLE_DETECTOR_H_ */
