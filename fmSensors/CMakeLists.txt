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

rosbuild_add_executable(sduimu_node src/global_sensors/sduimu_node.cpp)

rosbuild_add_executable(gps_node src/global_sensors/gps_node.cpp)

rosbuild_add_executable(sparkfun_razor_node src/global_sensors/SparkFun9DOF.cpp)
rosbuild_add_executable(sparkfun_razor_node src/global_sensors/sparkfun_razor_node.cpp)

include(CheckIncludeFiles)
check_include_files(/usr/lib/libLMS1xx.so HAVE_LMS111_LIB)
if(HAVE_LMS111_LIB)
	rosbuild_add_executable(lms111_node src/local_sensors/lms111_node.cpp)
	target_link_libraries(lms111_node /usr/lib/libLMS1xx.so)
else(HAVE_LMS111_LIB)
	message("Warning: no LMS1xx lib found; won't build LMS111 node")
endif(HAVE_LMS111_LIB)

rosbuild_add_executable(ll_encoder_node src/odometry_feedback/LeineLindeEncoder.cpp)
rosbuild_add_executable(ll_encoder_node src/odometry_feedback/ll_encoder_node.cpp)

include_directories(/usr/include/)
link_directories(/usr/lib/)


#Phidget Encoder (for CoroBot)
#rosbuild_add_executable(phidget_encoder_node src/odometry_feedback/phidget_encoder_node.cpp)
rosbuild_add_executable(corobot_wheel_feedback_node src/odometry_feedback/corobot_wheel_feedback_node.cpp)
rosbuild_add_executable(corobot_wheel_feedback_node src/odometry_feedback/corobot_wheel_feedback.cpp)
#target_link_libraries(phidget_encoder_node /usr/lib/libphidget21.so)

#Neato LIDAR (From Neato XV-15 robot vacuumcleaner)
rosbuild_add_library(neato_lidar_driver src/local_sensors/neato_lidar_driver.cpp)
rosbuild_add_executable(neato_lidar_node src/local_sensors/neato_lidar_node.cpp)
rosbuild_add_boost_directories()
rosbuild_link_boost(neato_lidar_node system)
target_link_libraries(neato_lidar_node neato_lidar_driver)

#Phidget 8/8/8 board (for CoroBot)
#rosbuild_add_executable(phidget_888_node src/system_feedback/phidget_888_node.cpp)
#target_link_libraries(phidget_888_node /usr/lib/libphidget21.so)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
