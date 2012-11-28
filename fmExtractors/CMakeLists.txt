cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#rosbuild_add_executable(lidar_row_detector_opencv src/lidar_feature_extractors/lidar_row_detect_opencv/maize_detection_node.cpp)
#rosbuild_add_executable(lidar_row_detector_opencv src/lidar_feature_extractors/lidar_row_detect_opencv/maize_detection.cpp)

rosbuild_add_executable(lidar_inrow_vehicle_detector src/lidar_feature_extractors/lidar_inrow_vehicle_detect/inrow_vehicle_detect_node.cpp)
rosbuild_add_executable(lidar_inrow_vehicle_detector src/lidar_feature_extractors/lidar_inrow_vehicle_detect/inrow_vehicle_detector.cpp)
rosbuild_add_executable(lidar_inrow_vehicle_detector src/lidar_feature_extractors/lidar_inrow_vehicle_detect/particle_filter.cpp)

#rosbuild_add_executable(gps_state src/global_pose_extractors/gps_state_node.cpp)
rosbuild_add_executable(imu_ypr src/imu_data_extractors/imu_ypr_extractor.cpp)

rosbuild_add_executable(simple_odom src/local_pose_extractors/simple_odom.cpp)
rosbuild_add_executable(simple_imu_odom src/local_pose_extractors/simple_imu_odom.cpp)

#Lidar obstacle avoidance and navigator node
rosbuild_add_executable(lidar_navigator src/lidar_feature_extractors/lidar_obstacle_avoidance_and_navigator/lidar_nav_node.cpp)
rosbuild_add_executable(lidar_navigator src/lidar_feature_extractors/lidar_obstacle_avoidance_and_navigator/lidar_navigator.cpp)
rosbuild_add_executable(lidar_navigator src/lidar_feature_extractors/lidar_obstacle_avoidance_and_navigator/pid_regulator.cpp)

rosbuild_add_executable(odometry_position_node src/odometry_position_extractor/odometry_position_extractor.cpp)

#rosbuild_add_executable(encoder_ackerman_to_pose src/local_pose_extractors/EncoderAckerman/encoder_ackerman_to_pose_node.cpp)
#rosbuild_add_executable(encoder_ackerman_to_pose src/local_pose_extractors/EncoderAckerman/PoseExtractorEncoderBase.cpp)
#rosbuild_add_executable(encoder_ackerman_to_pose src/local_pose_extractors/EncoderAckerman/PoseExtractorEncoderAckermanWithAngle.cpp)
#rosbuild_add_executable(encoder_ackerman_to_pose src/local_pose_extractors/EncoderAckerman/PoseExtractorEncoderAckermanWithAngleFWE.cpp)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

