/****************************************************************************
 # In row navigation
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
 # File: in_row_nav.cpp
 # Purpose: In-row navigation class
 # Project: Field Robot - Vehicle Interface Computer
 # Author: Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
 # Created: Jun 28, 2011 Søren Hundevadt Nielsen, Source written
 ****************************************************************************/

#include "in_row_nav.h"

IN_ROW_NAV::IN_ROW_NAV(double ap,double ai,double ad,double lp,double li,double ld){
	angle_regulator = PIDRegulator(ap,ai,ad);
	distance_regulator = PIDRegulator(lp,li,ld);
	publish = 1;
	nav_allow = false;
}

IN_ROW_NAV::~IN_ROW_NAV(){

}

void IN_ROW_NAV::maizehandler(const fmMsgs::vehicle_position maize_msg){
	if(nav_allow == true){
		float error_angle = maize_msg.position.th;
		if(error_angle < M_PI)
			error_angle = -2*M_PI + error_angle; 

		distance_regulator_output_ = distance_regulator.update(maize_msg.position.y-0.375,0);
		angle_regulator_output_ = angle_regulator.update(error_angle,0);

		ROS_INFO("Dist Error: %f Ang Error: %f",distance_regulator_output_,angle_regulator_output_);

		twist_msg.header.stamp = ros::Time::now();
		twist_msg.twist.linear.x=1.0;
		twist_msg.twist.angular.z=angle_regulator_output_ - distance_regulator_output_;
		twist_pub_.publish(twist_msg);
		publish = 1;
	}

	if(nav_allow == false){
		twist_msg.twist.linear.x=0;
		twist_msg.twist.angular.z=0;
		if (publish)
			twist_pub_.publish(twist_msg);
		publish = 0;
	}


}

void IN_ROW_NAV::allow_handler(const fmMsgs::row_nav_allow allow_msg){
	nav_allow = allow_msg.allow;
	ROS_INFO("%b", nav_allow);
}
