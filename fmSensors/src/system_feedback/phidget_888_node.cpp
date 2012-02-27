#include <string>
#include <sstream>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "fmMsgs/battery_power.h"
#include "fmMsgs/ir_distance.h"

#include <phidget21.h>

  /* Phidget */
  CPhidgetInterfaceKitHandle ifKit = 0;

  int  analog_inputs_value [8] = {0,0,0,0,0,0,0,0};
  bool m_validAnalogs = false;
  bool phidget888_connected;

  int batteryPort;
  int irFrontPort;
  int irBackPort;

  ros::Publisher powerdata_pub;
  ros::Publisher irData_pub;


int AttachHandler(CPhidgetHandle IFK, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName(IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);

	printf("%s %10d attached!\n", name, serialNo);

	return 0;
}

int DetachHandler(CPhidgetHandle IFK, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);

	printf("%s %10d detached!\n", name, serialNo);

	return 0;
}

int ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
	//printf("Error handled. %d - %s", ErrorCode, unknown);  //Do not print error info
	printf("Error handled. %d - %s \n", ErrorCode, unknown);
	return 0;
}

  /** @brief Description:
  Convert a voltage from the Phidgets 8/8/8 board into a distance, for the Infrared sensors
  return 25.4 meters as a distance if the value is out of range
  otherwise returns a valid distance in meters
  **/
  static float irVoltageToDistance(float volts)
  {
    int sensorValue=int(volts*200.0+0.5);
    float distanceInCm;
    if (sensorValue>=80 && sensorValue <= 500) //Outside of those bonds, the value is incorrect as our sensor can detect from 10cm to 80cm only
      {
        distanceInCm= 4800/(sensorValue-20);
      }
    else //out of bonds
      {
        distanceInCm = 2540; // 1000 inches in cm
      }
    return distanceInCm/100.0;
  }

  /**
   * @brief Publish the data on their corresponding topics
   */
  int publish_data(){

    if (m_validAnalogs){
          ////////////////////////////
          // Update power data
  	if(batteryPort != -1)
  	{
          	fmMsgs::battery_power powerdata;
          	powerdata.battery_volts = (float) (analog_inputs_value[batteryPort] - 500) * 0.0734;
		powerdata.header.stamp = ros::Time::now();
          	powerdata_pub.publish(powerdata);
  	}

          ////////////////////////////
          // Update IR data
  	if(irFrontPort != -1 || irBackPort != -1)
  	{
          	fmMsgs::ir_distance irData;
  		if(irFrontPort != -1)
  			irData.voltage1=(float) analog_inputs_value[irFrontPort] / 200.0;
  		else
  			irData.voltage1=0;
  		if(irBackPort != -1)
  			irData.voltage2=(float) analog_inputs_value[irBackPort] / 200.0;
  		else
  			irData.voltage2=0;
  		irData.range1=irVoltageToDistance(irData.voltage1);
  		irData.range2=irVoltageToDistance(irData.voltage2);
		irData.header.stamp = ros::Time::now();
  		irData_pub.publish(irData);
  	}

    }

      return 0;
  }

  //callback that will run if the sensor value changes by more than the OnSensorChange trigger.
  //Index - Index of the sensor that generated the event, Value - the sensor read value
  int SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
  {
  	//printf("Sensor: %d > Value: %d\n", Index, Value);

  	//sensorValue 0-1000 ==> 0-5V

      	analog_inputs_value[Index]=Value;      //SEGMENTATION FAULT!!

     	m_validAnalogs = true;

  	return 0;
  }

  int init_phidget888()
  {
  	int result, num_analog_inputs, num_digital_inputs;
  	const char *err;

  	//create the InterfaceKit object
  	CPhidgetInterfaceKit_create(&ifKit);

  	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.

  	CPhidget_set_OnDetach_Handler((CPhidgetHandle)ifKit, DetachHandler, NULL);
  	CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);

  	CPhidgetInterfaceKit_set_OnSensorChange_Handler (ifKit, SensorChangeHandler, NULL);


  	//open the interfacekit for device connections
  	CPhidget_open((CPhidgetHandle)ifKit, -1);

  	CPhidgetInterfaceKit_getInputCount(ifKit, &num_digital_inputs);
  	CPhidgetInterfaceKit_getSensorCount(ifKit, &num_analog_inputs);

  	printf("Waiting for interface kit to be attached....");
  	if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 1000)))
  	{
  		CPhidget_getErrorDescription(result, &err);
  		printf("Problem waiting for attachment: %s\n", err);
  		phidget888_connected = false;
  		return 0;
  	}

  	phidget888_connected = true;

  	CPhidgetInterfaceKit_setRatiometric(ifKit, 0);//

  	return 0;
  }

int main(int argc, char **argv)
{

  /* parameters */
  std::string power_data_pub_topic;
  std::string ir_data_pub_topic;
  int frequency;

  /* initialize ros usage */
  ros::init(argc, argv, "phidget888_node");

  /* private nodehandlers */
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  /* read parameters from ros parameter server if available otherwise use default values */
  n.param<std::string> ("ir_publisher_topic", ir_data_pub_topic, "ir_data"); //Specify the publisher name
  n.param<std::string> ("power_publisher_topic", power_data_pub_topic, "power_data"); //Specify the publisher name
  n.param<int> ("update_frequency", frequency, 10); //Update frequency
  n.param("battery_port", batteryPort, 0);
  n.param("ir_front_port", irFrontPort, 1);
  n.param("ir_back_port", irBackPort, 2);
  
  init_phidget888();

  irData_pub = n.advertise<fmMsgs::ir_distance>(ir_data_pub_topic, 1);
  powerdata_pub = n.advertise<fmMsgs::battery_power>(power_data_pub_topic, 1);

  ros::Rate loop_rate(frequency); //Encoder loop frequency

  while (ros::ok())
  {
    ros::spinOnce();

    publish_data();

    loop_rate.sleep();
  }

  ros::spin();

  printf("Closing...\n");
  CPhidget_close((CPhidgetHandle)ifKit); //Close encoder connection
  CPhidget_delete((CPhidgetHandle)ifKit); //Delete the encoder handle

  return 0;
}


