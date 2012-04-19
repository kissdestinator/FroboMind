/****************************************************************************
# In row navigation node
# Copyright (c) 2011 Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
*****************************************************************************
# File: in_row_node_node.cpp
# Purpose: In-row navigation node
# Project: Field Robot - Vehicle Interface Computer
# Author: Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
# Created: Jun 28, 2011 Søren Hundevadt Nielsen, Source written
****************************************************************************/
#include "in_row_nav.h"

int main(int argc, char **argv) {

	//Initialize ros usage
	ros::init(argc, argv, "inrow_navigation");

	//Create Nodehandlers
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	double angle_regulator_p;
	double angle_regulator_i;
	double angle_regulator_d;
	double linear_regulator_p;
	double linear_regulator_i;
	double linear_regulator_d;
	
	n.param<double> ("angular_p", angle_regulator_p, 0.2);
	n.param<double> ("angular_i", angle_regulator_i, 0);
	n.param<double> ("angular_d", angle_regulator_d, 0);
	n.param<double> ("linear_p", linear_regulator_p, 0.2);
	n.param<double> ("linear_i", linear_regulator_i, 0);
	n.param<double> ("linear_d", linear_regulator_d, 0);

	IN_ROW_NAV irn(angle_regulator_p,angle_regulator_i,angle_regulator_d,linear_regulator_p,linear_regulator_i,linear_regulator_d);

	n.param<std::string> ("maize_sub", irn.maize_sub_top_, "/fmExtractors/vehicle_position");
	n.param<std::string> ("twist_top", irn.twist_pub_top_, "/speed_from_joystick");
	n.param<std::string> ("allow_sub", irn.allow_sub_top_, "/allow");
	
	irn.nav_allow = false;

	irn.twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>(irn.twist_pub_top_.c_str(),1);
	irn.maize_row_sub_ = nh.subscribe<fmMsgs::vehicle_position>(irn.maize_sub_top_.c_str(),100,&IN_ROW_NAV::maizehandler, &irn);
	irn.allow_sub_ = nh.subscribe<fmMsgs::row_nav_allow>(irn.allow_sub_top_.c_str(),1,&IN_ROW_NAV::allow_handler,&irn);

	//Handle callbacks
	ros::spin();

}
