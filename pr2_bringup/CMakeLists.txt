# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_bringup)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rostest)

if(CATKIN_ENABLE_TESTING)
	# Tests that PR2 diagnostic analyzers configuration loads
	#DFS: commented out test that is timing out jenkins build
	#add_rostest(test/pr2_analyzers_load_test.launch)
endif()

catkin_package()

# install
install(DIRECTORY config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" 
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
install(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
   pr2.launch
   pr2_recalibrate.launch
   pr2-c2.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  # Tests that PR2 diagnostic analyzers configuration loads
  add_rostest(test/pr2_analyzers_load_test.launch)
  # Disable because pr2.launch needs to resolve c1/c2 hostname
  # roslaunch_add_file_check(pr2.launch ROS_ENV_LOADER=/opt/ros/$ENV{ROS_DISTRO}/env.sh)
  # roslaunch_add_file_check(pr2-c2.launch)
  roslaunch_add_file_check(pr2_recalibrate.launch)
endif()

