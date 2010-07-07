cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

# Tests that PR2 diagnostic analyzers configuration loads
rosbuild_add_rostest(test/pr2_analyzers_load_test.launch)
