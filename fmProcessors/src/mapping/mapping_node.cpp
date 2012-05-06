#include "mapping.h"

int main(int argc, char **argv) {

	//Initialize ros usage
	ros::init(argc, argv, "mapping");

	//Create Nodehandlers
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	double update;
	
	n.param<double> ("update", update, 50);
	n.param<std::

	MAPPING map();
	
	map.update_frequency = update;
	
	map.map_pub = nh.advertise<fmMsgs::map>(map.map_pub_top_.c_str(),1);
		

	//Go into mainloop
	map.main_loop();

}
