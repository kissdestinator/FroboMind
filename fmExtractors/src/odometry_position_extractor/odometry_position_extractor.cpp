#include <ros/ros.h>
#include <stdio.h>
#include "fmMsgs/odometry.h"
#include "fmMsgs/Vector3.h"
#include "math.h"
#include "fmMsgs/ypr.h"
#include "fmMsgs/kalman_output.h"
#include "fmMsgs/vehicle_coordinate.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


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
	
	ros::Publisher odom_pub2 = h.advertise<nav_msgs::Odometry>("/odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

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
		
	    double dt = (current_time - last_time).toSec();
  
		vx = (vl+vr)/2;
		vy = 0;
		vth = (vr-vl)/lengthBetweenTwoWheels; //angular velocity in radian per second. 
		
		//compute odometry in a typical way given the velocities of the robot

	    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
	    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
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

	    //since all odometry is 6DOF we'll need a quaternion created from yaw
	    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	    //first, we'll publish the transform over tf
	    geometry_msgs::TransformStamped odom_trans;
	    odom_trans.header.stamp = current_time;
	    odom_trans.header.frame_id = "odom";
	    odom_trans.child_frame_id = "base_link";

	    odom_trans.transform.translation.x = x;
	    odom_trans.transform.translation.y = y;
	    odom_trans.transform.translation.z = 0.0;
	    odom_trans.transform.rotation = odom_quat;

	    //send the transform
	    odom_broadcaster.sendTransform(odom_trans);

	    //next, we'll publish the odometry message over ROS
	    nav_msgs::Odometry odom;
	    odom.header.stamp = current_time;
	    odom.header.frame_id = "odom";

	    //set the position
	    odom.pose.pose.position.x = x;
	    odom.pose.pose.position.y = y;
	    odom.pose.pose.position.z = 0.0;
	    odom.pose.pose.orientation = odom_quat;

	    //set the velocity
	    odom.child_frame_id = "base_link";
	    odom.twist.twist.linear.x = vx;
	    odom.twist.twist.linear.y = vy;
	    odom.twist.twist.angular.z = vth;

	    //publish the message
	    odom_pub2.publish(odom);

	    coordi_pub.publish(coord_pub_msg);

	    odom_pub.publish(pub_msg);
	    
	    last_time = current_time;
	    
	    loop_rate.sleep();
		
	}
}
