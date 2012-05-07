#include "mission_control.h"

MISSION_CONTROL::MISSION_CONTROL(){
	current_path = 0;
	row_number = 1;
}

MISSION_CONTROL::~MISSION_CONTROL(){

}

void MISSION_CONTROL::main_loop(){
	ros::Rate loop_rate(update_frequency);
	fmMsgs::heading_order heading_msg;
	//my_position_x = map_offset_x + width_of_pots + 0.5 * width_of_rows + width_of_rows + width_of_pots;
	//my_position_y = map_offset_y;
	my_position_y = 0;
	my_position_x = 0;
	my_position_th = 0;
	current_state = IN_ROW;
	current_y_placement = BOTTOM;
	current_turn_direction = RIGHT;
	row_number = 2;

	while(ros::ok()){
		switch(current_state){
			case IN_ROW:
				generate_path_in_row();
				check_end_row();
				break;
			case EXIT_ROW:
				if(current_turn_direction == UNKNOWN)
					current_state = EXPLORER_MODE;
				else if(current_turn_direction == LEFT)
					generate_path_left_exit();
				else if(current_turn_direction == RIGHT)
					generate_path_right_exit();
				get_current_path();
				break;
			case FIND_ROW:
				if(current_turn_direction == LEFT)
					generate_path_left_enter();
				else if(current_turn_direction == RIGHT)
					generate_path_right_enter();
				get_current_path();
				break;
			case BLOCKED_ROW:
				break;
			case EXPLORER_MODE:
				break;
		}
		ROS_INFO("x: %f, y: %f, my_x %f, my_y: %f, my_th: %f, y_state: %d, turn_state: %d", path[0][0], path[1][0], my_position_x, my_position_y, my_position_th, current_y_placement, current_turn_direction);
		heading_msg.orientation = get_new_heading();
		heading_pub.publish(heading_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void MISSION_CONTROL::map_callback(nav_msgs::OccupancyGrid msg){
	//my_position_x = msg.x;
	//my_position_y = msg.y;
	//my_position_th = msg.th;
}

void MISSION_CONTROL::p_filter_callback(fmMsgs::vehicle_position msg){
	my_position_x = msg.position.x;
	my_position_y = msg.position.y;
	my_position_th = msg.position.th;
}

double MISSION_CONTROL::get_new_heading(){
	/*
	 * find the delta heading from the robot's own heading, to the heading of the line from the robot to the point.
	 */
	double a(0), b(0);
	a = path[0][current_path] - my_position_x;
	b = path[1][current_path] - my_position_y;
	if(a == 0)
		a += 0.0001;

	double path_heading = (2 * M_PI - (atan2(a,b)));



	if((my_position_th < M_PI && path_heading < M_PI) || (my_position_th > M_PI && path_heading > M_PI)){
		if(my_position_th > path_heading)
			path_heading = my_position_th - path_heading;
		else
			path_heading = path_heading - my_position_th;
	}
	else
		if(my_position_th > path_heading)
			path_heading =  my_position_th - path_heading;
		else
			path_heading =  path_heading - my_position_th;

	return path_heading;
}

void MISSION_CONTROL::get_current_path(){
	/*
	 * Calculate whether the robot is close enough to the current point.
	 */
	if(current_state == EXIT_ROW){
		if(abs(my_position_x - path[0][current_path]) < path[2][current_path] && abs(my_position_y - path[1][current_path]) < path[2][current_path]){
			current_state = FIND_ROW;
			row_number++;
		}
	}
	else if(current_state == FIND_ROW)
		if(abs(my_position_x - path[0][current_path]) < path[2][current_path] && abs(my_position_y - path[1][current_path]) < path[2][current_path]){
			current_state = IN_ROW;
			if(current_y_placement == BOTTOM)
				current_y_placement = TOP;
			else
				current_y_placement = BOTTOM;

			if(current_turn_direction == RIGHT)
				current_turn_direction = LEFT;
			else
				current_turn_direction = RIGHT;
		}
}

void MISSION_CONTROL::generate_path_in_row(){

	path[0][0] = map_offset_x + row_number * (width_of_rows + width_of_pots) - (width_of_rows * 0.5);
	path[1][0] = map_offset_y + length_of_rows + row_exit_length;
	path[2][0] = point_proximity_treshold;

}

void MISSION_CONTROL::generate_path_right_exit(){
	if (current_y_placement == TOP){
		path[0][0] = map_offset_x + row_number * (width_of_pots + width_of_rows) + 0.5 * width_of_pots;
		path[1][0] = map_offset_y + length_of_rows + row_exit_length;
		path[2][0] = point_proximity_treshold;
	}
	else{
		path[0][0] = map_offset_x + row_number * (width_of_pots + width_of_rows) + 0.5 * width_of_pots;
		path[1][0] = map_offset_y - row_exit_length;
		path[2][0] = point_proximity_treshold;
	}

}

void MISSION_CONTROL::generate_path_left_exit(){
	if (current_y_placement == TOP){
		path[0][0] = map_offset_x + width_of_pots + ((width_of_pots + width_of_rows) * no_of_rows) - row_number * (width_of_pots + width_of_rows) + 0.5 * width_of_pots;
		path[1][0] = map_offset_y + length_of_rows + row_exit_length;
		path[2][0] = point_proximity_treshold;
	}
	else{
		path[0][0] = map_offset_x + row_number * (width_of_pots + width_of_rows) + 0.5 * width_of_pots;
		path[1][0] = map_offset_y - row_exit_length;
		path[2][0] = point_proximity_treshold;
	}

}

void MISSION_CONTROL::check_end_row(){
	//End of row upwards
	if(my_position_y > (map_offset_y + length_of_rows - end_row_limit))
		if((my_position_th > 1.5 * M_PI) || (my_position_th < 0.5 * M_PI))
			current_state = EXIT_ROW;

	//End of row downwards
	if(my_position_y < (map_offset_y + end_row_limit))
		if((my_position_th < 1.5 * M_PI) && (my_position_th > 0.5 * M_PI))
			current_state = EXIT_ROW;
}

void MISSION_CONTROL::generate_path_left_enter(){
	if(current_y_placement == BOTTOM){
		path[0][0] = map_offset_x + row_number * (width_of_rows + width_of_pots) + width_of_pots + (0.5 * width_of_rows);
		path[1][0] = map_offset_y;
		path[2][0] = point_proximity_treshold;
	}
	else if(current_y_placement == TOP){
		path[0][0] = map_offset_x + width_of_pots + ((width_of_pots + width_of_rows) * no_of_rows) - row_number * (width_of_pots + width_of_rows) - 0.5 * width_of_rows;
		path[1][0] = map_offset_y + length_of_rows;
		path[2][0] = point_proximity_treshold;
	}
}

void MISSION_CONTROL::generate_path_right_enter(){
	if(current_y_placement == BOTTOM){
		path[0][0] = map_offset_x + width_of_pots + ((width_of_pots + width_of_rows) * no_of_rows) - row_number * (width_of_pots + width_of_rows) - 0.5 * width_of_rows;
		path[1][0] = map_offset_y;
		path[2][0] = point_proximity_treshold;
	}
	else if(current_y_placement == TOP){
		path[0][0] = map_offset_x + row_number * (width_of_rows + width_of_pots) + width_of_pots + (0.5 * width_of_rows);
		path[1][0] = map_offset_y + length_of_rows;
		path[2][0] = point_proximity_treshold;
	}
}


