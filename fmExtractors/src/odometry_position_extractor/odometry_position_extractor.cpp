#include <ros/ros.h>
#include <stdio.h>
#include "fmMsgs/odometry.h"
#include "fmMsgs/Vector3.h"
#include "math.h"
#include "fmMsgs/ypr.h"


using namespace std;

bool start;
double x,y,th,vl,vr,vx,vy,vth,xr,xl, lxr, lxl, offset;
ros::Time current_time, last_time;
double lengthBetweenTwoWheels = 0.21+0.03;
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_position_extractor");
	
	ros::NodeHandle h;
	
	vl = vr = vy = vx = th = x = y = vth = xr = xl = lxl = lxr = 0;
	left_last_time = right_last_time = ros::Time::now();
	start = true;

	ros::Subscriber sub_left = h.subscribe("/fmSensors/left_odometry", 1, left_callback);
	ros::Subscriber sub_right = h.subscribe("/fmSensors/right_odometry", 1, right_callback);
	
   	fmMsgs::Vector3 pub_msg;

	ros::Rate loop_rate(50);

        ros::Publisher odom_pub = h.advertise<fmMsgs::Vector3>("xyz_position", 1); 

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
	    double delta_th = (vth * dt); 

	    x += delta_x;
	    y += delta_y;
	    th += delta_th;
	    
	    pub_msg.x = x;
	    pub_msg.y = y;
	    pub_msg.th = th;

	    odom_pub.publish(pub_msg);
	    
	    last_time = current_time;
	    
	    loop_rate.sleep();
		
	}
}
