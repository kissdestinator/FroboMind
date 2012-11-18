#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "fmMsgs/motor_power.h"
#include "fmMsgs/desired_speed.h"
#include "fmMsgs/odometry.h"
#include "motor_controller.h"
#include "fmMsgs/warhorse_state.h"

#include <sstream>

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "fmPointsMap");
  ros::NodeHandle n;
  ros::Publisher action_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
