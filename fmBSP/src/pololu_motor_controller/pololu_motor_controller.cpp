#include "pololu_motor_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>

PololuMotorController::PololuMotorController() {

}

void PololuMotorController::callbackHandler(const fmMsgs::motor_powerConstPtr& msg)
{
    int desired_speed_left = (msg->power_left +1)*127;
    int desired_speed_right = (msg->power_right +1)*127;
    char pololu_data[6] = {255,0,desired_speed_left,255,1,desired_speed_right};
    std::string data;
    data.assign(pololu_data,6);
    serial_msg.data = data;

    pololu_pub.publish(serial_msg);
}

