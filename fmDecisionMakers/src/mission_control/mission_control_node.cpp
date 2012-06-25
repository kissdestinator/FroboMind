#include "mission_control.h"

int main(int argc, char **argv) {

	//Initialize ros usage
	ros::init(argc, argv, "mission_control");

	//Create Nodehandlers
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	
	MISSION_CONTROL mc;
	
	n.param<double> ("update", mc.update_frequency, 50);
	n.param<std::string> ("Heading_Pub_Top", mc.heading_pub_top, "/fmDecisionMakers/Heading");
	n.param<std::string> ("Map_Sub_Top", mc.map_sub_top, "/fmExtractors/map");
	n.param<std::string> ("P_Filter_Sub_Top", mc.p_filter_sub_top, "/fmExtractors/vehicle_position");
	n.param<std::string> ("viz_pub_top", mc.viz_pub_top, "/fmDecisionMakers/viz_route");
	n.param<std::string> ("orders_string", mc.filename, "direction.txt");
	
	n.param<double> ("Length_of_rows", mc.length_of_rows, 3);
	n.param<double> ("Width_of_rows", mc.width_of_rows, 0.75);
	n.param<double> ("No_of_rows", mc.no_of_rows, 3);
	n.param<double> ("Map_offset_x", mc.map_offset_x, 10);
	n.param<double> ("Map_offset_y", mc.map_offset_y, 10);
	n.param<double> ("Point_proximity_treshold", mc.point_proximity_treshold, 0.1);
	n.param<double> ("Width_of_pots", mc.width_of_pots, 0.18);
	n.param<double> ("row_exit_length", mc.row_exit_length, 0.5);
	n.param<double> ("End_row_limit", mc.end_row_limit, 0.5);
	n.param<int> ("start_turn_direction", mc.direction, 0);
	n.param<int> ("Task_no", mc.task, 2);
	n.param<bool> ("set_simulation", mc.simulation, false);

	if(mc.direction == 0)
		mc.current_turn_direction = mc.LEFT;
	if(mc.direction == 1)
		mc.current_turn_direction = mc.RIGHT;
	if(mc.direction == 2)
		mc.current_turn_direction = mc.UNKNOWN;
	
	if(mc.simulation == false){
		mc.map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(mc.map_sub_top.c_str(),1,&MISSION_CONTROL::map_callback, &mc);
		mc.p_filter_sub = nh.subscribe<fmMsgs::vehicle_position>(mc.p_filter_sub_top.c_str(),1,&MISSION_CONTROL::p_filter_callback, &mc);
	}
	else{
		mc.client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
		mc.pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	}


	mc.viz_pub = nh.advertise<visualization_msgs::Marker>(mc.viz_pub_top.c_str(),1);
	mc.heading_pub = nh.advertise<fmMsgs::heading_order>(mc.heading_pub_top.c_str(),1);


	//Go into mainloop
	mc.main_loop();

}
