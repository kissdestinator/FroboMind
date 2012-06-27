#include "mission_control.h"

MISSION_CONTROL::MISSION_CONTROL(){
	current_path = 0;
	current_smoothed_path = 0;
	row_number = 1;
	blocked = false;
	start_smooth = 0;
	last_i = 0;

}

MISSION_CONTROL::~MISSION_CONTROL(){

}

void MISSION_CONTROL::main_loop(){
	ros::Rate loop_rate(update_frequency);
	fmMsgs::heading_order heading_msg;
	my_position_y = 10;
	my_position_x = 10;
	my_position_th = 0;
	current_state = IN_ROW;
	current_y_placement = TOP;
	current_turn_direction = RIGHT;
	row_number = 1;

	nav_msg.no_of_rows = no_of_rows;
	nav_msg.row_length = length_of_rows;
	nav_msg.row_offset_x = map_offset_x;
	nav_msg.row_offset_y = map_offset_y;
	nav_msg.row_spacing = width_of_rows;
	nav_msg.row_width = width_of_pots;
	start_x = 10;
	start_y = 10;

	get_file_path();
	make_path_from_orders();

	while(ros::ok()){
		if(simulation == true){
			get_pos_from_sim();
		}
		if(warhorse_state.task_state == warhorse_state.MANUAL_DRIVE){
			start_x = map_offset_x + width_of_pots + 0.5 * width_of_rows;
			start_y = map_offset_y;
			row_number = 1;
			filename = filename_task_1_left;
		}
		if(warhorse_state.task_state == warhorse_state.TASK1LEFT){
			start_x = map_offset_x + width_of_pots + 0.5 * width_of_rows;
			start_y = map_offset_y;
			row_number = 1;
			filename = filename_task_1_left;
		}
		else if(warhorse_state.task_state == warhorse_state.TASK1RIGHT){
			start_x = map_offset_x + ((no_of_rows - 1) * (width_of_pots + width_of_rows)) - 0.5 * width_of_rows;
			start_y = map_offset_y;
			row_number = no_of_rows -1;
			filename = filename_task_1_right;
		}
		else if(warhorse_state.task_state == warhorse_state.TASK2){
			filename = filename_task_2;
			start_x = map_offset_x + ((int)((no_of_rows - 1) / 2) * (width_of_pots + width_of_rows)) - 0.5 * width_of_rows;
			start_y = map_offset_y;
			row_number = (int)(no_of_rows / 2);
		}

		get_file_path();
		check_current_marker();
		make_smoothed_path(path[0][current_path], path[1][current_path],path[2][current_path]);
		heading_msg.orientation = get_new_heading_smooth();


		//ROS_INFO("x: %f, y: %f, ppt: %f, current_smoothed_path: %d, my_pos_x: %f, my_pos_y: %f", smoothed_path[0][current_smoothed_path],smoothed_path[1][current_smoothed_path],smoothed_path[2][current_smoothed_path], current_smoothed_path, my_position_x, my_position_y);

		if(simulation == true){
			//calcAndPublishSpeedSim(heading_msg.orientation, 0.5);
			get_new_headnig_quat();
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


		nav_msg.start_x = start_x;
		nav_msg.start_y = start_y;
		nav_spec_pub.publish(nav_msg);

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

void MISSION_CONTROL::state_callback(fmMsgs::warhorse_state msg){
	if(msg.drive_state == msg.STOP){
		current_smoothed_path = 0;
		current_path = 0;
	}

	task_state = msg.task_state;
	warhorse_state = msg;
	make_path_from_orders();
}

void MISSION_CONTROL::blocked_callback(fmMsgs::blocked_row msg){
	if(msg.blocked == true && warhorse_state.task_state == warhorse_state.TASK2){
		blocked_row = current_path;
		invert_path();
	}
}

void MISSION_CONTROL::invert_path(){
	if(path[0][current_path] > path[0][current_path-1])
		way_turn = 1;
	else
		way_turn = -1;

	path[0][current_path] = path[0][current_path-1];
	path[1][current_path] = path[1][current_path-1];
	path[2][current_path] = path[2][current_path-1];

	for(int i = (blocked_row + 1)*2; i < 2*sizeof(in_turns) ; i+=2){
		way_turn = way_turn * -1;
		ROS_INFO("%c", in_turns[i/2]);

		if(in_turns[i/2] == 'S'){
			generate_path_in_row();
			path[0][i+1] = path[0][i];
			path[1][i+1] = path[1][i];
			path[2][i+1] = path[2][i];
			path[0][0] = start_x;
			path[1][0] = start_y;
			path[2][0] = 10;
		}

		else if(in_turns[i/2] == 'F'){
			path[0][i] = -1;
			ROS_INFO("x: %f, y: %f, p: %f", path[0][i],path[1][i],path[2][i]);
			break;
		}

		else if(in_turns[i/2] == '0'){
			path[0][i] = path[0][i-1];
			if(way_turn < 0)
				path[1][i] = map_offset_y - row_exit_length;
			else
				path[1][i] = map_offset_y + length_of_rows + row_exit_length;
			path[2][i] = point_proximity_treshold;

			path[0][i+1] = path[0][i];
			if(way_turn > 0)
				path[1][i+1] = map_offset_y - row_exit_length;
			else
				path[1][i+1] = map_offset_y + length_of_rows + row_exit_length;
			path[2][i+1] = point_proximity_treshold;

		}

		else if(in_turns[i/2] == 'R'){

			path[0][i] = path[0][i-1] + (double)in_path[i/2] * (width_of_rows + width_of_pots) * way_turn;
			if(way_turn < 0)
				path[1][i] = map_offset_y - row_exit_length;
			else
				path[1][i] = map_offset_y + length_of_rows + row_exit_length;

			path[2][i] = point_proximity_treshold;


			path[0][i+1] = path[0][i];
			if(way_turn > 0)
				path[1][i+1] = map_offset_y - row_exit_length;
			else
				path[1][i+1] = map_offset_y + length_of_rows + row_exit_length;
			path[2][i+1] = point_proximity_treshold;

		}

		else if(in_turns[i/2] == 'L'){

			path[0][i] = path[0][i-1] - (double)in_path[i/2] * (width_of_rows + width_of_pots) * way_turn;
			if(way_turn < 0)
				path[1][i] = map_offset_y - row_exit_length;
			else
				path[1][i] = map_offset_y + length_of_rows + row_exit_length;

			path[2][i] = point_proximity_treshold;


			path[0][i+1] = path[0][i];
			if(way_turn > 0 )
				path[1][i+1] = map_offset_y - row_exit_length;
			else
				path[1][i+1] = map_offset_y + length_of_rows + row_exit_length;
			path[2][i+1] = point_proximity_treshold;
		}


		ROS_INFO("x: %f, y: %f, p: %f", path[0][i],path[1][i],path[2][i]);
		ROS_INFO("x: %f, y: %f, p: %f", path[0][i+1],path[1][i+1],path[2][i+1]);

	}
}

void MISSION_CONTROL::check_current_marker(){
	if((fabs(my_position_x - path[0][current_path] ) < path[2][current_path] )&&( fabs(my_position_y - path[1][current_path]) < path[2][current_path])){
		current_path++;
		current_smoothed_path = 0;
		make_smoothed_path(path[0][current_path], path[1][current_path],path[2][current_path]);
	}

	if((fabs(my_position_x - smoothed_path[0][current_smoothed_path] ) < smoothed_path[2][current_smoothed_path] )&&( fabs(my_position_y - smoothed_path[1][current_smoothed_path]) < smoothed_path[2][current_smoothed_path]))
		current_smoothed_path++;

}

double MISSION_CONTROL::get_new_heading_smooth(){
	/*
	 * find the delta heading from the robot's own heading, to the heading of the line from the robot to the point.
	 */

	if(path[0][current_path] == -1)
		return -1;

	double a(0), b(0);
	a = smoothed_path[0][current_smoothed_path] - my_position_x;
	b = smoothed_path[1][current_smoothed_path] - my_position_y;
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

double MISSION_CONTROL::get_new_headnig_quat(){
	double a(0), b(0);
	a = path[0][current_path] - my_position_x;
	b = path[1][current_path] - my_position_y;
	if(a == 0)
		a += 0.0001;

	double path_heading = (2 * M_PI - (atan2(a,b)));

	while(path_heading > (2 * M_PI))
		path_heading -= (2* M_PI);

	path_heading = (path_heading - M_PI/2);

	while(path_heading < (2 * M_PI))
		path_heading += (2* M_PI);

	while(path_heading > (2 * M_PI))
		path_heading -= (2* M_PI);

	//q_path.setEulerZYX(path_heading,0,0);

	//ROS_INFO("turn_angle: %f, path_heading: %f, q_angle: %f",q.angle(q_path), path_heading, q.getAngle());

	return q.angle(q_path);
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
				in_turns[temp_count];
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
	in_turns[temp_count] = c;
	/*
	for(int i = 0; i < 29; i++)
		ROS_INFO("turn: %c", in_turns[i]); */
}

void MISSION_CONTROL::get_pos_from_sim(){
	gazebo_msgs::GetModelState getmodelstate;
	getmodelstate.request.model_name = "robot_description";
	client.call(getmodelstate);
	my_position_x = getmodelstate.response.pose.position.x + 10;
	my_position_y = getmodelstate.response.pose.position.y + 10;
	double q0(getmodelstate.response.pose.orientation.x), q1(getmodelstate.response.pose.orientation.y), q2(getmodelstate.response.pose.orientation.z), q3(getmodelstate.response.pose.orientation.w);

	q.setX(q0);
	q.setY(q1);
	q.setZ(q2);
	q.setW(q3);
	//ROS_INFO("angle: %f, x: %f, y: %f, z: %f, w: %f", (double)q.getAngle(), q0,q1,q2,q3);
	/*my_position_th = (double)q.getAngle() ; //+ M_PI/2;
	if(my_position_th > 2*M_PI)
		my_position_th-= 2*M_PI;

	// Convert quaternion to RPY.

	    double roll, pitch, yaw;
	    tf::quaternionMsgToTF(msg->orientation, q);
	    btMatrix3x3(q).getRPY(roll, pitch, yaw);
	    ROS_DEBUG("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
	/*
	btQuaternion q;
	double roll, pitch, yaw;
	tf::quaternionMsgToTF(getmodelstate.response.pose.orientation, q);
	btMatrix3x3(q).getRPY(roll, pitch, yaw);
	ROS_INFO("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
	*/

	//ROS_INFO("My pos th: %f", my_position_th);

}

void MISSION_CONTROL::make_path_from_orders(){
	way_turn = 1;
	path[0][0] = 1;
	path[1][0] = 1;
	path[2][0] = 1;
	ROS_INFO("Start");

	for(int i = 0; i < 2*sizeof(in_turns) ; i+=2){
		way_turn = way_turn * -1;
		ROS_INFO("%c", in_turns[i/2]);
		
		if(in_turns[i/2] == 'S'){
			generate_path_in_row();
			path[0][i+1] = path[0][i];
			path[1][i+1] = path[1][i];
			path[2][i+1] = path[2][i];
			path[0][0] = start_x;
			path[1][0] = start_y;
			path[2][0] = 10;
		}
		
		else if(in_turns[i/2] == 'F'){
			path[0][i] = -1;
			//ROS_INFO("x: %f, y: %f, p: %f", path[0][i],path[1][i],path[2][i]);
			break;
		}
		
		else if(in_turns[i/2] == '0'){
			path[0][i] = path[0][i-1];
			if(way_turn < 0)
				path[1][i] = map_offset_y - row_exit_length;
			else
				path[1][i] = map_offset_y + length_of_rows + row_exit_length;
			path[2][i] = point_proximity_treshold;
			
			path[0][i+1] = path[0][i];
			if(way_turn > 0)
				path[1][i+1] = map_offset_y - row_exit_length;
			else
				path[1][i+1] = map_offset_y + length_of_rows + row_exit_length;
			path[2][i+1] = point_proximity_treshold;
			
		}
		
		else if(in_turns[i/2] == 'R'){
			
			path[0][i] = path[0][i-1] + (double)in_path[i/2] * (width_of_rows + width_of_pots) * way_turn;
			if(way_turn < 0)
				path[1][i] = map_offset_y - row_exit_length;
			else
				path[1][i] = map_offset_y + length_of_rows + row_exit_length;
			
			path[2][i] = point_proximity_treshold;
			
			
			path[0][i+1] = path[0][i];
			if(way_turn > 0)
				path[1][i+1] = map_offset_y - row_exit_length;
			else
				path[1][i+1] = map_offset_y + length_of_rows + row_exit_length;
			path[2][i+1] = point_proximity_treshold;
			
		}
		
		else if(in_turns[i/2] == 'L'){
			
			path[0][i] = path[0][i-1] - (double)in_path[i/2] * (width_of_rows + width_of_pots) * way_turn;
			if(way_turn < 0)
				path[1][i] = map_offset_y - row_exit_length;
			else
				path[1][i] = map_offset_y + length_of_rows + row_exit_length;
			
			path[2][i] = point_proximity_treshold;
			
			
			path[0][i+1] = path[0][i];
			if(way_turn > 0 )
				path[1][i+1] = map_offset_y - row_exit_length;
			else
				path[1][i+1] = map_offset_y + length_of_rows + row_exit_length;
			path[2][i+1] = point_proximity_treshold;
		}
		

		//ROS_INFO("x: %f, y: %f, p: %f", path[0][i],path[1][i],path[2][i]);
		//ROS_INFO("x: %f, y: %f, p: %f", path[0][i+1],path[1][i+1],path[2][i+1]);

	}

	if(start_smooth == 0){
		current_path++;
		make_smoothed_path(path[0][current_path], path[1][current_path],path[2][current_path]);
		start_smooth = 1;
	}

}

void MISSION_CONTROL::calcAndPublishSpeedSim(double turn_angle, double velocity)
{
	/*
	 * geometry_msgs::Twist cmdvel_;
			cmdvel_.angular.z = heading_msg.orientation;
			cmdvel_.linear.x = 0;
	        pub_.publish(cmdvel_);
	 */
	double vel = 0;
	double ang_vel = 0;

	// Calculate velocity
	if (abs(turn_angle) < 15 * DEG2RAD)
		vel = velocity;
	else if (abs(turn_angle) < 90 * DEG2RAD)
		vel = velocity * (90 - (abs(turn_angle)*RAD2DEG)) / 75;

	//Calculate Angular velocity
	//ang_vel = pid_ang_vel.update(-turn_angle,0);

	ang_vel = turn_angle * 10;
	if (abs(ang_vel) > 2)
		{
			if(ang_vel < 0)
				ang_vel = -2;
			else
				ang_vel = 2;
		}

	geometry_msgs::Twist twistSim;
	twistSim.linear.x = vel;
	twistSim.angular.z = ang_vel;
	//if (twist.twist.linear.x != 0 && twist.twist.angular.z != 0)
	pub_.publish(twistSim);

	//ROS_INFO("Turn Angle: %.3f, Velocity: %.3f, Angular Velocity: %.3f",turn_angle,vel,ang_vel);
}

void MISSION_CONTROL::make_smoothed_path(double x, double y, double p_thresh){
	double distance_x = fabs(path[0][current_path-1] - path[0][current_path]);
	double distance_y = fabs(path[1][current_path-1] - path[1][current_path]);
	double direction = -1;
	int i = 0;
	int i_count = 0;

	if(distance_x > 1){
		if(path[0][current_path-1] < path[0][current_path])
			direction = 1;
		for( i = 0; i < distance_x; i++){
			smoothed_path[0][i] = path[0][current_path-1] + i*direction;
			smoothed_path[1][i] = y;
			smoothed_path[2][i] = p_thresh;
			//ROS_INFO("X_direction. Point: x: %f, y: %f",smoothed_path[0][i], smoothed_path[1][i] );
			i_count++;
		}
	}
	else if(distance_y > 1){
		if(path[1][current_path-1] < path[1][current_path])
			direction = 1;
		for( i = 0; i < distance_y;  i++){
			smoothed_path[0][i] = x;
			smoothed_path[1][i] = path[1][current_path-1] + i*direction;
			smoothed_path[2][i] = p_thresh;
			//ROS_INFO("Y_direction. Point: x: %f, y: %f",smoothed_path[0][i], smoothed_path[1][i] );
			i_count++;
		}
	}
	else{
		i = 1;
		smoothed_path[0][0] = x;
		smoothed_path[1][0] = y;
		smoothed_path[2][0] = p_thresh;
		//ROS_INFO("CLose enough. Point: x: %f, y: %f, distance_x: %f, distance_y: %f, my_posx: %f, my_posy: %f:",smoothed_path[0][0], smoothed_path[1][0], distance_x, distance_y,my_position_x, my_position_y );
	}
	smoothed_path[0][i] = path[0][current_path];
	smoothed_path[1][i] = path[1][current_path];
	smoothed_path[2][i] = path[2][current_path];

	smoothed_path[0][i+1] = -1;
	smoothed_path[1][i+1] = -1;
	smoothed_path[2][i+1] = -1;
	i = last_i;
	markerarray.markers.clear();

	viz_pub_marker.publish(markerarray);

 	i = current_smoothed_path;
	while(smoothed_path[0][i] != 0){
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = i;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = smoothed_path[1][i];
		marker.pose.position.y = smoothed_path[0][i];
		marker.pose.position.z = 0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		markerarray.markers.push_back(marker);
		ROS_INFO("x: %f, y: %f", smoothed_path[0][i],smoothed_path[1][i] );
		i++;
	}
	last_i = i;

	viz_pub_marker.publish(markerarray);



}
