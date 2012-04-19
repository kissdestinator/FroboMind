/****************************************************************************
 # In row navigation
 # Copyright (c) 2011 Thomas Iversen
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
 # File: Path_Controller.h
 # Purpose: Path controller class
 # Project: Field Robot - Warhorse
 # Author: Thomas Iversen <thive09@student.sdu.dj>
 # Created: Apr 18, 2012
 ****************************************************************************/

#ifndef PATH_CONTROLLER_H_
#define PATH_CONTROLLER_H_

#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "pid_regulator.h"
#include "fmMsgs/path.h"
#include "fmMsgs/kalman_output.h"
#include "fmMsgs/Vector3.h"
#include "fmMsgs/vehicle_position.h"
#include "math.h"

class PATH_CONTROLLER {

private:
	PIDRegulator angle_regulator;
    PIDRegulator distance_regulator;
    
    geometry_msgs::TwistStamped twist_msg;

	double angle_regulator_output_;
	double distance_regulator_;

	std::vector<double> point_x;
	std::vector<double> point_y;

	double x,y,th_kal, th_row, th_points, error_angle;
	ros::Time last_time;
	

public:
	double update_frequency;
	ros::Publisher twist_pub;
	ros::Subscriber kalman_sub;
	ros::Subscriber path_sub;
	ros::Subscriber xy_sub;
	ros::Subscriber row_sub;
	
	std::string kalman_sub_top_;
	std::string twist_pub_top_;
	std::string path_sub_top_;
	std::string xy_sub_top_;
	std::string row_sub_top_;

	
	PATH_CONTROLLER(double ap,double ai,double ad,double lp,double li,double ld);
	virtual ~PATH_CONTROLLER();

	void orientation_handler(const fmMsgs::kalman_output kal_msg);
	void path_handler(const fmMsgs::path path_msg);
	void xy_handler(const fmMsgs::Vector3 xy_msg);
	void row_handler(const fmMsgs::vehicle_position row_msg);
	void main_loop();
	
};

#endif /* PATH_CONTROLLER_H_ */
