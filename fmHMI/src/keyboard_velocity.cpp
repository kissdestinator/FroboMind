#include <ros/ros.h>
#include "fmMsgs/desired_speed.h"
#include <stdio.h>
#include <fmMsgs/Joy.h>
#include <signal.h>
#include "fmMsgs/row_nav_allow.h"
#include <termios.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_ENTER 0x0A

ros::Publisher vel_pub;
ros::Publisher allow_pub;

int kfd = 0;
struct termios cooked, raw;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboardVelocity");

  ros::NodeHandle h;
  ros::NodeHandle nh("~");
  std::string velocity_pub_topic;
  std::string allow_pub_topic;
  nh.param<std::string>("velocity_pub_topic", allow_pub_topic, "allow");
  nh.param<std::string>("allow_pub_topic", velocity_pub_topic, "/speed_from_joystick1");
  vel_pub = h.advertise<fmMsgs::desired_speed>(velocity_pub_topic, 1);
  allow_pub = h.advertise<fmMsgs::row_nav_allow>(allow_pub_topic, 1);

  char c;
  bool dirty=false;
  fmMsgs::desired_speed hastighed;
  fmMsgs::row_nav_allow allow_pub_;

  allow_pub_.allow = false;
  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");
	ros::Rate loop_rate(50);

  for(;;)
  {	
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        hastighed.speed_right = -1.0;
        hastighed.speed_left = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        hastighed.speed_right = 1.0;
        hastighed.speed_left = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        hastighed.speed_right = 1.0;
        hastighed.speed_left = 1.0;
        dirty = true;
	allow_pub_.allow = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        hastighed.speed_right = -1.0;
        hastighed.speed_left = -1.0;
	allow_pub_.allow = false;
        dirty = true;
        break;
	
      default:
	hastighed.speed_right = 0;
	hastighed.speed_left = 0;
	break;

    }
    ROS_INFO("value: %c", c); 

    c = 0x00;
    float scale = 0.5;
    hastighed.speed_left *= scale;
    hastighed.speed_right *= scale;
    if(dirty)
    	vel_pub.publish(hastighed);

     allow_pub.publish(allow_pub_);  

    loop_rate.sleep(); 
    
  }


  return(0);
}
