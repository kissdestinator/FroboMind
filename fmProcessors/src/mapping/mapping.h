
#ifndef MAPPING_H_
#define MAPPING_H_

#include <ros/ros.h>
#include "math.h"
#include "fmMsgs/map.h"

class MAPPING {

private:
	double[][] map;

public:
	double update_frequency;
	ros::Subscriber map_sub;
	
	std::string map_sub_top_;

	
	MAPPING();
	virtual ~MAPPING();
	
};

#endif /* PATH_CONTROLLER_H_ */
