
#include "mapping.h"

MAPPING::MAPPING(){
	ros::Rate loop_rate(update_frequency);
	map = double[5000][10000];
}

MAPPING::~MAPPING(){

}

void MAPPING::mainloop(){
	
	loop_rate.sleep();
	ros::spinOnce();
}

