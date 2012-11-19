#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "fmMsgs/motor_power.h"
#include "fmMsgs/desired_speed.h"
#include "fmMsgs/odometry.h"
#include "motor_controller.h"
#include "fmMsgs/warhorse_state.h"

int main(int argc, char **argv)
{
	/* initialize ros usage */
	ros::init(argc, argv, "motor_controller");

	/* parameters */
	std::string motor_power_pub_topic;
	std::string navigation_speed_sub_topic;
	std::string left_odometry_sub_topic;
	std::string right_odometry_sub_topic;
	double max_speed, p_left, i_left, d_left, p_right, i_right, d_right, max_acceleration, max_deacceleration, windup_left, windup_right;

	/* private nodehandlers */
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	/* read parameters from ros parameter server if available otherwise use default values */
	n.param<std::string> ("motor_power_topic", motor_power_pub_topic, "motor_power"); //Specify the publisher name
	n.param<std::string> ("navigation_speed_subscriber_topic", navigation_speed_sub_topic, "/navigation_vel"); //Specify the publisher name
	n.param<std::string> ("left_odo_subscriber_topic", left_odometry_sub_topic, "/fmSensors/left_odometry"); //Specify the publisher name
	n.param<std::string> ("right_odo_subscriber_topic", right_odometry_sub_topic, "/fmSensors/right_odometry"); //Specify the publisher name
	n.param<double> ("maximum_speed", max_speed, 1); //Specify the maximum speed in m/s
	n.param<double> ("maximum_acceleration", max_acceleration, 0); //Specify the maximum acceleration in m/s² (0 = inf acceleration)
	n.param<double> ("maximum_deacceleration", max_deacceleration, 0); //Specify the maximum deacceleration in m/s² (0 = inf deacceleration)
	n.param<double> ("p_left", p_left, 1);
	n.param<double> ("i_left", i_left, 0);
	n.param<double> ("d_left", d_left, 0);
	n.param<double> ("p_right", p_right, 1);
	n.param<double> ("i_right", i_right, 0);
	n.param<double> ("d_right", d_right, 0);
	n.param<double> ("windup_left", windup_left, 1000);
	n.param<double> ("windup_right", windup_right, 1000);

	MotorController mc = MotorController(p_left,i_left,d_left,p_right,i_right,d_right);

	mc.setMaxSpeed(max_speed);
	mc.setMaxAcceleration(max_acceleration);
	mc.setMaxDeacceleration(max_deacceleration);
	mc.setIWindupLimitLeft(windup_left);
	mc.setIWindupLimitRight(windup_right);

	mc.navigation_speed_sub = n.subscribe<geometry_msgs::Twist>(navigation_speed_sub_topic.c_str(),1,&MotorController::navigationSpeedHandler,&mc);
	mc.left_odo_sub = n.subscribe(left_odometry_sub_topic.c_str(), 1, &MotorController::leftMotorHandler,&mc);
	mc.right_odo_sub = n.subscribe(right_odometry_sub_topic.c_str(), 1, &MotorController::rightMotorHandler,&mc);
	mc.motor_power_pub = n.advertise<fmMsgs::motor_power>(motor_power_pub_topic.c_str(), 1);

	ros::spin(); // wait for callbacks

	return 0;
}
