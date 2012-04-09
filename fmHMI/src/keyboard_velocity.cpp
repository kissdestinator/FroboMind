#include <ros/ros.h>
#include "fmMsgs/desired_speed.h"
#include <stdio.h>
#include <fmMsgs/Joy.h>
#include <signal.h>
#include <termios.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

ros::Publisher vel_pub;

int kfd = 0;
struct termios cooked, raw;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboardVelocity");

  ros::NodeHandle h;
  ros::NodeHandle nh("~");
  std::string velocity_pub_topic;
  nh.param<std::string>("velocity_pub_topic", velocity_pub_topic, "/speed_from_joystick");
  vel_pub = h.advertise<fmMsgs::desired_speed>(velocity_pub_topic, 1);

  char c;
  bool dirty=false;
  fmMsgs::desired_speed hastighed;

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

    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        hastighed.speed_right = 0.0;
        hastighed.speed_left = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        hastighed.speed_right = 1.0;
        hastighed.speed_left = 0.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        hastighed.speed_right = 1.0;
        hastighed.speed_left = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        hastighed.speed_right = -1.0;
        hastighed.speed_left = -1.0;
        dirty = true;
        break;
      default:
	hastighed.speed_right = 0;
	hastighed.speed_left = 0;
	break;

    }

    c = 0x00;

    vel_pub.publish(hastighed);   

	    loop_rate.sleep(); 
    
  }


  return(0);
}
