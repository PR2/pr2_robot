cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

# roslaunch checks
rosbuild_add_roslaunch_check(pr2_default_controllers.launch)
rosbuild_add_roslaunch_check(pr2_jt_controllers.launch)
