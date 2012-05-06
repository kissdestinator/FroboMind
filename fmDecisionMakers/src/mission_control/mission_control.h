#ifndef MISSION_CONTROL_H_
#define MISSION_CONTROL_H_

#include <ros/ros.h>
#include "fmMsgs/heading_order.h"
#include "fmMsgs/Vector3.h"


class MISSION_CONTROL {

private:
	double path[3][10];
	double my_position_x, my_position_y, my_position_th;
	int current_path;
	double get_new_heading();
	void get_current_path();
	void generate_path();
	enum state { IN_ROW, EXIT_ROW, FIND_ROW, BLOCKED_ROW, EXPLORER_MODE};
	enum y_placement {BOTTOM, TOP};
	y_placement current_y_placement;
	double row_number;
	void check_end_row();
	void generate_path_in_row();
	void generate_path_right_exit();
	void generate_path_left_exit();
	void generate_path_left_enter();
	void generate_path_right_enter();

public:
	std::string map_sub_top;
	std::string p_filter_sub_top;
	std::string heading_pub_top;
	
	enum turn_direction {RIGHT, LEFT, UNKNOWN};
	state current_state;
	turn_direction current_turn_direction;

	double length_of_rows, width_of_rows, width_of_pots, no_of_rows, map_offset_x, map_offset_y;
	double end_row_limit, point_proximity_treshold, row_exit_length;
	int direction;
	ros::Subscriber map_sub;
	ros::Subscriber p_filter_sub;
	ros::Publisher heading_pub;
	
	double update_frequency;

	MISSION_CONTROL();
	virtual ~MISSION_CONTROL();
	
	void main_loop();
	void map_callback(fmMsgs::Vector3 msg);
	void p_filter_callback(fmMsgs::Vector3 msg);
	
};

#endif /* MISSION_CONTROL_H_ */
