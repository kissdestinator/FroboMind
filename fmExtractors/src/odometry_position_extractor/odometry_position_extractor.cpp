#include <ros/ros.h>
#include <stdio.h>
#include "fmMsgs/odometry.h"
#include "fmMsgs/Vector3.h"
#include "math.h"
#include "fmMsgs/ypr.h"
#include "fmMsgs/kalman_output.h"
#include "fmMsgs/vehicle_coordinate.h"


using namespace std;

bool start;
double x,y,th,vl,vr,vx,vy,vth,xr,xl, lxr, lxl, offset, kalman_th, odo_th;
ros::Time current_time, last_time;
double lengthBetweenTwoWheels = 0.39;
ros::Time right_time, right_last_time, left_time, left_last_time;

void right_callback(fmMsgs::odometry odo_msg_in)
{
	right_last_time = right_time;
	right_time = odo_msg_in.header.stamp;
	lxr = xr;
	xr = odo_msg_in.position;
	return;
}

void left_callback(fmMsgs::odometry odo_msg_in)
{	
	left_last_time = left_time;
	left_time = odo_msg_in.header.stamp;
	lxl = xl;
	xl = odo_msg_in.position;
	return;
}

void kalman_callback(fmMsgs::kalman_output msg)
{
	kalman_th = msg.yaw;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_position_extractor");
	
	ros::NodeHandle h;
	
	vl = vr = vy = vx = th = x = y = vth = xr = xl = lxl = lxr = kalman_th = odo_th = 0;
	left_last_time = right_last_time = ros::Time::now();
	start = true;

	ros::Subscriber sub_left = h.subscribe("/fmSensors/left_odometry", 1, left_callback);
	ros::Subscriber sub_right = h.subscribe("/fmSensors/right_odometry", 1, right_callback);
	ros::Subscriber sub_kalman = h.subscribe("/fmProcessors/Kalman_AngularVelocity", 1, kalman_callback);
	
   	fmMsgs::Vector3 pub_msg;
	fmMsgs::vehicle_coordinate coord_pub_msg;

	ros::Rate loop_rate(50);

        ros::Publisher odom_pub = h.advertise<fmMsgs::Vector3>("xyz_position", 1); 
        ros::Publisher coordi_pub = h.advertise<fmMsgs::vehicle_coordinate>("vehicle_coordinate", 1); 

	while(h.ok()){

	    ros::spinOnce();
		
		

		vl = (xl - lxl)/(left_time - left_last_time).toSec(); 
		vr = (xr - lxr)/(right_time - right_last_time).toSec(); 
		
		current_time = ros::Time::now();    
		vx = (vl+vr)/2;
		vy = 0;
		vth = (vr-vl)/lengthBetweenTwoWheels; //angular velocity in radian per second. 
		
			//compute odometry in a typical way given the velocities of the robot
	    double dt = (current_time - last_time).toSec();
	    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
	    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;

	    ROS_DEBUG("VL: %f, VR: %f. XL: %f, LXL: %f, left_time: %f, left_lefttime: %f, vx: %f", vl, vr, xl, lxl, (double)left_time.toSec(), (double)left_last_time.toSec(), vx);
	    
	    double delta_th = (vth * dt); 

	    x += delta_x;
	    y += delta_y;
	    th = kalman_th;
	    odo_th += delta_th;
	    
	    pub_msg.x = vx;
	    pub_msg.y = vy;
	    pub_msg.th = vth;
	    pub_msg.header.stamp = ros::Time::now();

	    coord_pub_msg.x = x;
	    coord_pub_msg.y = y;
	    coord_pub_msg.th = th;

	    coordi_pub.publish(coord_pub_msg);

	    odom_pub.publish(pub_msg);
	    
	    last_time = current_time;
	    
	    loop_rate.sleep();
		
	}
}
