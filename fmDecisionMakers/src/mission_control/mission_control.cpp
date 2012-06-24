#include "mission_control.h"

MISSION_CONTROL::MISSION_CONTROL(){
	current_path = 0;
	row_number = 1;
	blocked = false;
}

MISSION_CONTROL::~MISSION_CONTROL(){

}

void MISSION_CONTROL::main_loop(){
	ros::Rate loop_rate(update_frequency);
	fmMsgs::heading_order heading_msg;
	my_position_y = 0;
	my_position_x = 0;
	my_position_th = 0;
	current_state = IN_ROW;
	current_y_placement = TOP;
	current_turn_direction = RIGHT;
	row_number = 1;

	while(ros::ok()){
		if(simulation == true){
			get_pos_from_sim();
		}
		if(task==1){
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
			heading_msg.orientation = get_new_heading();
		}
		else if(task == 2){
			get_file_path();
			make_path_from_orders();
			check_current_marker();
			heading_msg.orientation = get_new_heading();

			/*
			for(int i = 0; i < 29; i++)
				ROS_INFO("path0: %f, path1: %f", path[0][i], path[1][i]); 
				*/
		}

		heading_pub.publish(heading_msg);
		//ROS_INFO("x: %f, y: %f, my_x %f, my_y: %f, my_th: %f, y_state: %d, heading: %.3f, turn_state: %d, row_number: %f, state: %d, if: %s, task: %f", path[0][0], path[1][0], my_position_x, my_position_y, my_position_th, current_y_placement, heading_msg.orientation, current_turn_direction, row_number, current_state, (double)task);

		visualization_msgs::Marker marker;

		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = path[1][current_path];
		marker.pose.position.y = path[0][current_path];
		marker.pose.position.z = 0;
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;

		viz_pub.publish(marker);

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void MISSION_CONTROL::map_callback(nav_msgs::OccupancyGrid msg){
	//my_position_x = msg.x;
	//my_position_y = msg.y;
	//my_position_th = msg.th;r
}

void MISSION_CONTROL::p_filter_callback(fmMsgs::vehicle_position msg){
		my_position_x = msg.position.x;
		my_position_y = msg.position.y;
		my_position_th = msg.position.th;
}

void MISSION_CONTROL::check_current_marker(){
	if((fabs(my_position_x - path[0][current_path] ) < path[2][current_path] )&&( fabs(my_position_y - path[1][current_path]) < path[2][current_path])){
		current_path++;
	}
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

	while(path_heading > (2 * M_PI))
		path_heading -= (2* M_PI);

	//ROS_INFO("%f", path_heading);

	if((my_position_th < M_PI && path_heading < M_PI) || (my_position_th > M_PI && path_heading > M_PI))
		path_heading = path_heading - my_position_th;
	else
		if(my_position_th < path_heading)
			path_heading = path_heading - (my_position_th + (2* M_PI));
		else
			path_heading = path_heading - (my_position_th - (2* M_PI));



	while((path_heading > (2*M_PI) )||( path_heading < 0)){
		if(path_heading > 2 * M_PI)
			path_heading =  path_heading - (2 * M_PI);
		if(path_heading < 0)
			path_heading =  path_heading + (2 * M_PI);
	}
	return path_heading;
}

void MISSION_CONTROL::get_current_path(){
	/*
	 * Calculate whether the  robot is close enough to the current point.
	 */

	if(current_state == EXIT_ROW){
		if((fabs(my_position_x - path[0][current_path] ) < path[2][current_path] )&&( fabs(my_position_y - path[1][current_path]) < path[2][current_path])){
			current_state = FIND_ROW;
			row_number++;
		}
	}

	else if(current_state == FIND_ROW)
		if((fabs(my_position_x - path[0][current_path] ) < path[2][current_path] )&&( fabs(my_position_y - path[1][current_path]) < path[2][current_path])){
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
	if(current_y_placement == BOTTOM){
		path[0][0] = map_offset_x + row_number * (width_of_rows + width_of_pots) - (width_of_rows * 0.5);
		path[1][0] = map_offset_y - row_exit_length;
		path[2][0] = point_proximity_treshold;
	}
	else if (current_y_placement == TOP){
		path[0][0] = map_offset_x + row_number * (width_of_rows + width_of_pots) - (width_of_rows * 0.5);
		path[1][0] = map_offset_y + length_of_rows + row_exit_length;
		path[2][0] = point_proximity_treshold;
	}

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
		if(current_y_placement == TOP){
			current_state = EXIT_ROW;
		}

	//End of row downwards
	if(my_position_y < (map_offset_y + end_row_limit))
		if(current_y_placement == BOTTOM){
			current_state = EXIT_ROW;
		}
}

void MISSION_CONTROL::generate_path_left_enter(){
	if(current_y_placement == BOTTOM){
		path[0][0] = map_offset_x + row_number * (width_of_rows + width_of_pots) - (0.5 * width_of_rows);
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
		path[0][0] = map_offset_x + row_number * (width_of_rows + width_of_pots) - (0.5 * width_of_rows);
		path[1][0] = map_offset_y;
		path[2][0] = point_proximity_treshold;
	}
	else if(current_y_placement == TOP){
		path[0][0] = map_offset_x + row_number * (width_of_rows + width_of_pots) - (0.5 * width_of_rows);
		path[1][0] = map_offset_y + length_of_rows;
		path[2][0] = point_proximity_treshold;
	}
}

void MISSION_CONTROL::get_file_path(){
	
	char c;
	std::ifstream file;
	file.open(filename.c_str());
	if(file.fail()){
		ROS_INFO("Something went wrong, mother fucker! Filename: %s", filename.c_str());
		exit(1);
	}
	temp_count = 0;
	temp_type = 1;
	file.get(c);
	while(!file.eof()){
		switch (c)
		{
			case 'S':
				temp_count = 0;
				in_path[temp_count] = -1;
				in_turns[temp_count] = c;
				break;
				
			case '-':
				temp_count++;
				break;
			
			case ' ':
				break;
				
			case 'F':
				file >> c;
				break;
				
			case '0':
				in_path[temp_count] = c - 48;
				in_turns[temp_count] = c;
				break;
	        default:
	        	if(temp_type == 1){
	        		in_path[temp_count] = c - 48;
	        		temp_type = 0;
	        	}
	        	else{
	        		in_turns[temp_count] = c;
	        		temp_type = 1;
	        	}
				break;
	        }
	       file >> c;
	  }  
	/*
	for(int i = 0; i < 29; i++)
		ROS_INFO("turn: %c", in_turns[i]); */
}

void MISSION_CONTROL::get_pos_from_sim(){
	gazebo_msgs::GetModelState getmodelstate;
	getmodelstate.request.model_name = "robot_description";
	client.call(getmodelstate);
	my_position_x = getmodelstate.response.pose.position.x;
	my_position_y = getmodelstate.response.pose.position.y;
	double q0(getmodelstate.response.pose.orientation.x), q1(getmodelstate.response.pose.orientation.y), q2(getmodelstate.response.pose.orientation.z), q3(getmodelstate.response.pose.orientation.w);
	my_position_th = atan2(2*q0*q3-2*q1*q2, 1-2*(q0*q0)-2*q2*q2);

	/*
	btQuaternion q;
	q.getRPY()
	double roll, pitch, yaw;
tf::quaternionMsgToTF(getmodelstate.response.pose.orientation, q);
	btMatrix3x3(q).getRPY(roll, pitch, yaw);
	ROS_INFO("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);

	//ROS_INFO("My pos th: %f", my_position_th);*/

}

void MISSION_CONTROL::make_path_from_orders(){

	temp = 1;
	path[0][0] = 1;
	path[1][0] = 1;
	path[2][0] = 1;
	for(int i = 0; i < 2*sizeof(in_turns)-1 ; i+=2){

		temp = temp * -1;
		
		if(in_turns[i/2] == 'S'){
			generate_path_in_row();
			path[0][i+1] = path[0][i];
			path[1][i+1] = path[1][i];
			path[2][i+1] = path[2][i];
		}
		
		else if(in_turns[i/2] == 'F'){
			break;
		}
		
		else if(in_turns[i/2] == '0'){
			path[0][i] = path[0][i-1];
			if(temp < 0)
				path[1][i] = map_offset_y - row_exit_length;
			else
				path[1][i] = map_offset_y + length_of_rows + row_exit_length;
			path[2][i] = point_proximity_treshold;
			
			path[0][i+1] = path[0][i];
			if(temp > 0)
				path[1][i+1] = map_offset_y - row_exit_length;
			else
				path[1][i+1] = map_offset_y + length_of_rows + row_exit_length;
			path[2][i+1] = point_proximity_treshold;
			
		}
		
		else if(in_turns[i/2] == 'R'){
			
			path[0][i] = path[0][i-1] + (double)in_path[i/2] * (width_of_rows + width_of_pots) * temp;
			if(temp < 0)
				path[1][i] = map_offset_y - row_exit_length;
			else
				path[1][i] = map_offset_y + length_of_rows + row_exit_length;
			
			path[2][i] = point_proximity_treshold;
			
			
			path[0][i+1] = path[0][i];
			if(temp > 0)
				path[1][i+1] = map_offset_y - row_exit_length;
			else
				path[1][i+1] = map_offset_y + length_of_rows + row_exit_length;
			path[2][i+1] = point_proximity_treshold;
			
		}
		
		else if(in_turns[i/2] == 'L'){
			
			path[0][i] = path[0][i-1] - (double)in_path[i/2] * (width_of_rows + width_of_pots) * temp;
			if(temp < 0)
				path[1][i] = map_offset_y - row_exit_length;
			else
				path[1][i] = map_offset_y + length_of_rows + row_exit_length;
			
			path[2][i] = point_proximity_treshold;
			
			
			path[0][i+1] = path[0][i];
			if(temp > 0 )
				path[1][i+1] = map_offset_y - row_exit_length;
			else
				path[1][i+1] = map_offset_y + length_of_rows + row_exit_length;
			path[2][i+1] = point_proximity_treshold;
		}
		
	}
}
