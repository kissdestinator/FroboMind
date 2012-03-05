#include <string>
#include <sstream>
#include <stdio.h>

#include "ros/ros.h"
#include "fmMsgs/encoder.h"
#include "std_msgs/String.h"

#include <phidget21.h>

int CCONV AttachHandler(CPhidgetHandle ENC, void *userptr)
{
        int serialNo;
        CPhidget_DeviceID deviceID;
        int i, inputcount;

        CPhidget_getSerialNumber(ENC, &serialNo);

        //Retrieve the device ID and number of encoders so that we can set the enables if needed
        CPhidget_getDeviceID(ENC, &deviceID);
        CPhidgetEncoder_getEncoderCount((CPhidgetEncoderHandle)ENC, &inputcount);
        printf("Encoder %10d attached! \n", serialNo);

        //the 1047 requires enabling of the encoder inputs, so enable them if this is a 1047    
        if (deviceID == PHIDID_ENCODER_HS_4ENCODER_4INPUT)
        {
                printf("Encoder requires Enable. Enabling inputs....\n");
                for (i = 0 ; i < inputcount ; i++)
                        CPhidgetEncoder_setEnabled((CPhidgetEncoderHandle)ENC, i, 1);
        }
        return 0;
}


int CCONV DetachHandler(CPhidgetHandle ENC, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(ENC, &serialNo);
	printf("Encoder %10d detached! \n", serialNo);

	return 0;
}

int CCONV ErrorHandler(CPhidgetHandle ENC, void *userptr, int ErrorCode, const char *Description)
{
	printf("Error handled. %d - %s \n", ErrorCode, Description);

	return 0;
}

int CCONV InputChangeHandler(CPhidgetEncoderHandle ENC, void *usrptr, int Index, int State)
{
	printf("Input #%i - State: %i \n", Index, State);

	return 0;
}

CPhidgetEncoderHandle init_encoder()
{
	int result;
	const char *err;

	//Declare an encoder handle
	CPhidgetEncoderHandle encoder = 0;

	//create the encoder object
	CPhidgetEncoder_create(&encoder);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)encoder, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)encoder, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)encoder, ErrorHandler, NULL);

	CPhidget_open((CPhidgetHandle)encoder, -1);

	//get the program to wait for an encoder device to be attached
	printf("Waiting for encoder to be attached....");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)encoder, 2000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}

	return encoder;
}

int main(int argc, char **argv)
{
  /* ros messages */
  fmMsgs::encoder enc_msg;

  /* parameters */
  std::string publisher_topic;
  int direction;
  int serial;
  int frequency;

  /* initialize ros usage */
  ros::init(argc, argv, "phidget_encoder_node");
  ros::Publisher enc_publisher;

  /* private nodehandlers */
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  /* read parameters from ros parameter server if available otherwise use default values */
  n.param<std::string> ("publisher_topic", publisher_topic, "enc_msg"); //Specify the publisher name
  n.param<int> ("update_frequency", frequency, 10); //Update frequency
  n.param<int> ("direction", direction, 1); //Set the direction (-1 if the encoder is mounted backwards)

  enc_publisher = nh.advertise<fmMsgs::encoder> (publisher_topic.c_str(), 1);

  CPhidgetEncoderHandle encoder = init_encoder();
  if (!encoder)
  {
     ROS_ERROR("Connection failed");
     CPhidget_close((CPhidgetHandle)encoder); //Close encoder connection
     CPhidget_delete((CPhidgetHandle)encoder); //Delete the encoder handle
     return 1;
  }
  
  CPhidget_getSerialNumber((CPhidgetHandle)encoder, &serial);

  std::string serialString;
  std::stringstream out;
  out << serial;
  serialString = out.str();

  ros::Rate loop_rate(frequency); //Encoder loop frequency

  int encoder_value;

  while (ros::ok())
  {
    ros::spinOnce();
    
    CPhidgetEncoder_getPosition(encoder, 0, &encoder_value); //Get encoderposition
    
    enc_msg.encoderticks = encoder_value;
    enc_msg.header.stamp = ros::Time::now();
    enc_msg.header.frame_id = serialString;

    enc_publisher.publish(enc_msg); //Publish message

    loop_rate.sleep();
  }

  CPhidget_close((CPhidgetHandle)encoder); //Close encoder connection
  CPhidget_delete((CPhidgetHandle)encoder); //Delete the encoder handle

  return 0;
}


