/****************************************************************************
 # File: Path_Controller.cpp
 # Purpose: Path controller class
 # Project: Field Robot - Warhorse
 # Author: Thomas Iversen <thive09@student.sdu.dk>
 # Created: Apr 18, 2012
 ****************************************************************************/

#include "Path_Controller.h"

PATH_CONTROLLER::PATH_CONTROLLER(double ap,double ai,double ad,double lp,double li,double ld){
	angle_regulator = PIDRegulator(ap,ai,ad);
	distance_regulator = PIDRegulator(lp,li,ld);
	x = y = 0;
	point_x.push_back((double)0); point_x.push_back((double)1);
	point_y.push_back((double)0); point_y.push_back((double)1);
}

PATH_CONTROLLER::~PATH_CONTROLLER(){

}

void PATH_CONTROLLER::main_loop(){
	ros::Rate loop_rate(update_frequency);
	while(ros::ok){
		double x_koeff = (point_y[1] - point_y[0]) / point_x[1] - point_x[0];
		th_points = atan(1/x_koeff);

		error_angle = th_kal - th_points;

		twist_msg.twist.linear.x = 0.5;
		twist_msg.twist.angular.z = error_angle;

		if(point_x[1] - point_x[0] < 0.10 && point_y[1] - point_y[0] < 0.10){
			twist_msg.twist.linear.x = 0;
			twist_msg.twist.angular.z = 0;
		}

		twist_pub.publish(twist_msg);

		ROS_DEBUG("Point 1: %f, %f - Point 2: %f, %f - X: %f - Error_angle: %f - Th_points: %f", point_x[0], point_y[0], point_x[1], point_y[1], x_koeff, error_angle, th_points);

		loop_rate.sleep();
		ros::spinOnce();
	}


}

void PATH_CONTROLLER::orientation_handler(const fmMsgs::kalman_output kal_msg){
	th_kal = kal_msg.yaw;
}

void PATH_CONTROLLER::path_handler(const fmMsgs::path path_msg){
	point_x = path_msg.pointx;
	point_y = path_msg.pointy;
	x = y = 0;
}

void PATH_CONTROLLER::xy_handler(const fmMsgs::Vector3 xy_msg){
	double vx = xy_msg.x;
	double vy = xy_msg.y;
	ros::Time current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th_kal) - vy * sin(th_kal)) * dt;
    double delta_y = (vx * sin(th_kal) + vy * cos(th_kal)) * dt;
    last_time = current_time;
    x += delta_x;
    y += delta_y;
    point_x[0] = x;
    point_y[0] = y;
}

void PATH_CONTROLLER::row_handler(const fmMsgs::vehicle_position row_msg){
	th_row = row_msg.position.th;
}


