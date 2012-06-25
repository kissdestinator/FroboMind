#ifndef MISSION_CONTROL_H_
#define MISSION_CONTROL_H_

#include <ros/ros.h>
#include "fmMsgs/heading_order.h"
#include "fmMsgs/Vector3.h"
#include "fmMsgs/vehicle_position.h"
#include "nav_msgs/OccupancyGrid.h"
#include "cmath"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "fstream"
#include "string"
#include "cctype"
#include "iostream"
#include "gazebo_msgs/GetModelState.h"
#include "/opt/ros/electric/stacks/geometry_experimental/tf2/include/LinearMath/btQuaternion.h"
#include "/opt/ros/electric/stacks/geometry_experimental/tf2/include/LinearMath/btMatrix3x3.h"

#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI

class MISSION_CONTROL {

private:
	int temp_count;
	int temp_type;
	double temp;
	int start_smooth;
	double path[3][30];
	double in_path[30];
	char in_turns[30];
	double smoothed_path[3][30];
	double my_position_x, my_position_y, my_position_th;
	int current_path;
	int current_smoothed_path;
	bool blocked;
	double get_new_heading();
	void get_current_path();
	void generate_path();
	enum state { IN_ROW, EXIT_ROW, FIND_ROW, BLOCKED_ROW, EXPLORER_MODE};
	enum y_placement {BOTTOM, TOP};
	y_placement current_y_placement;
	double row_number;
	btQuaternion q;
	btQuaternion q_path;
	void check_end_row();
	void generate_path_in_row();
	void generate_path_right_exit();
	void generate_path_left_exit();
	void generate_path_left_enter();
	void generate_path_right_enter();
	void get_file_path();
	void make_path_from_orders();
	void check_current_marker();
	void get_pos_from_sim();
	void calcAndPublishSpeedSim(double turn_angle, double velocity);
	double get_new_headnig_quat();

	void make_smoothed_path(double x, double y, double p_thresh);

public:
	std::string map_sub_top;
	std::string p_filter_sub_top;
	std::string heading_pub_top;
	std::string viz_pub_top;
	std::string viz_pub_top_marker;
	int task;
	std::string filename;
	bool simulation;
	
	enum turn_direction {RIGHT, LEFT, UNKNOWN};
	state current_state;
	turn_direction current_turn_direction;

	double length_of_rows, width_of_rows, width_of_pots, no_of_rows, map_offset_x, map_offset_y;
	double end_row_limit, point_proximity_treshold, row_exit_length;
	int direction;
	ros::Subscriber map_sub;
	ros::Subscriber p_filter_sub;
	ros::Publisher heading_pub;
	ros::Publisher viz_pub;
	ros::Publisher viz_pub_marker;
	ros::ServiceClient client;
	ros::Publisher pub_;
	
	double update_frequency;

	MISSION_CONTROL();
	virtual ~MISSION_CONTROL();
	
	void main_loop();
	void map_callback(nav_msgs::OccupancyGrid msg);
	void p_filter_callback(fmMsgs::vehicle_position msg);
	
};

#endif /* MISSION_CONTROL_H_ */
