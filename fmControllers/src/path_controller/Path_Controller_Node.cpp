/****************************************************************************
# File: Path_Controller_node.cpp
# Purpose: Path controller node
# Project: Field Robot - Warhorse
# Author: Thomas Iversen <thive09@student.sdu.dk>
# Created: Apr 18, 2012 
****************************************************************************/
#include "Path_Controller.h"

int main(int argc, char **argv) {

	//Initialize ros usage
	ros::init(argc, argv, "path_controller");

	//Create Nodehandlers
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	
	double angle_regulator_p;
	double angle_regulator_i;
	double angle_regulator_d;
	double linear_regulator_p;
	double linear_regulator_i;
	double linear_regulator_d;
	double update;
	
	n.param<double> ("angular_p", angle_regulator_p, 0.2);
	n.param<double> ("angular_i", angle_regulator_i, 0);
	n.param<double> ("angular_d", angle_regulator_d, 0);
	n.param<double> ("linear_p", linear_regulator_p, 0.2);
	n.param<double> ("linear_i", linear_regulator_i, 0);
	n.param<double> ("linear_d", linear_regulator_d, 0);
	n.param<double> ("update", update, 50);

	PATH_CONTROLLER pth(angle_regulator_p,angle_regulator_i,angle_regulator_d,linear_regulator_p,linear_regulator_i,linear_regulator_d);

	pth.update_frequency = update;

	n.param<std::string> ("kalman_sub", pth.kalman_sub_top_, "/fmProcessors/Kalman_AngularVelocity");
	n.param<std::string> ("twist_top", pth.twist_pub_top_, "/speed_from_joystick");
	n.param<std::string> ("allow_sub", pth.path_sub_top_, "/Path");
	n.param<std::string> ("xy_sub_top", pth.xy_sub_top_, "/fmExtractors/xyz_position");
	n.param<std::string> ("row_sub_top", pth.row_sub_top_, "/fmExtractors/vehicle_position");
	
	pth.twist_pub = nh.advertise<geometry_msgs::TwistStamped>(pth.twist_pub_top_.c_str(),1);
	pth.kalman_sub = nh.subscribe<fmMsgs::kalman_output>(pth.kalman_sub_top_.c_str(),1,&PATH_CONTROLLER::orientation_handler,&pth);
	pth.path_sub = nh.subscribe<fmMsgs::path>(pth.path_sub_top_.c_str(),1,&PATH_CONTROLLER::path_handler,&pth);
	pth.xy_sub = nh.subscribe<fmMsgs::Vector3>(pth.xy_sub_top_.c_str(),1,&PATH_CONTROLLER::xy_handler,&pth);
	pth.row_sub = nh.subscribe<fmMsgs::vehicle_position>(pth.row_sub_top_.c_str(),1,&PATH_CONTROLLER::row_handler,&pth);


	//Go into mainloop
	pth.main_loop();

}
