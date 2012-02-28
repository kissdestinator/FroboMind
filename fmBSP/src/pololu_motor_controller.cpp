#include "pololu_motor_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>

#define base 0x378           /* printer port base address */
#define value 255            /* numeric value to send to printer port */

PololuMotorController::PololuMotorController() {

}

void PololuMotorController::callbackHandler(const fmMsgs::desired_speedConstPtr& msg)
{
    int desired_speed_left = (msg->speed_left +1)*127;
    int desired_speed_right = (msg->speed_right +1)*127;
    char pololu_data[6] = {255,0,desired_speed_left,255,1,desired_speed_right};
    std::string data;
    data.assign(pololu_data,6);
    serial_msg.data = data;

   if (ioperm(base,1,1))
    fprintf(stderr, "Couldn't get the port at %x\n", base), exit(1);

   outb(value, base);

    pololu_pub.publish(serial_msg);
}

