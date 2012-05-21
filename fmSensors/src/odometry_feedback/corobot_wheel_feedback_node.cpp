#include <string>
#include <sstream>
#include <stdio.h>

#include "ros/ros.h"
#include "fmMsgs/encoder.h"
#include "fmMsgs/odometry.h"
#include "std_msgs/String.h"

#include "corobot_wheel_feedback.h"

int main(int argc, char **argv)
{
  /* ros messages */
  fmMsgs::odometry odo_msg;

  /* parameters */
  std::string left_odo_pub_topic;
  std::string right_odo_pub_topic;
  std::string left_odo_pub_topic_window;
  std::string right_odo_pub_topic_window;
  std::string encoder1;
  std::string encoder2;

  /* initialize ros usage */
  ros::init(argc, argv, "corobot_wheel_feedback_node");

  /* private nodehandlers */
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  /* Instanciate objects */
  CorobotWheelFeedback cwf;

  /* read parameters from ros parameter server if available otherwise use default values */
  n.param<std::string> ("left_odo_pub_topic", left_odo_pub_topic, "left_odometry"); //Specify the publisher name
  n.param<std::string> ("right_odo_pub_topic", right_odo_pub_topic, "right_odometry"); //Specify the publisher name
  n.param<std::string> ("left_odo_pub_topic_window", left_odo_pub_topic_window, "left_odometry_window"); //Specify the publisher name
  n.param<std::string> ("right_odo_pub_topic_window", right_odo_pub_topic_window, "right_odometry_window"); //Specify the publisher name
  n.param<std::string> ("encoder1", encoder1, "/fmSensors/encoder1");
  n.param<std::string> ("encoder2", encoder2, "/fmSensors/encoder2");

  cwf.left_odometry_pub = nh.advertise<fmMsgs::odometry> (left_odo_pub_topic.c_str(), 1);
  cwf.right_odometry_pub = nh.advertise<fmMsgs::odometry> (right_odo_pub_topic.c_str(), 1);
  cwf.left_odometry_pub_window = nh.advertise<fmMsgs::odometry> (left_odo_pub_topic_window.c_str(), 1);
  cwf.right_odometry_pub_window = nh.advertise<fmMsgs::odometry> (right_odo_pub_topic_window.c_str(), 1);
  
  cwf.enc1_sub = nh.subscribe(encoder1, 1, &CorobotWheelFeedback::callbackHandlerEncoder1, &cwf);
  cwf.enc2_sub = nh.subscribe(encoder2, 1, &CorobotWheelFeedback::callbackHandlerEncoder2, &cwf);

  ros::spin();

  return 0;
}


