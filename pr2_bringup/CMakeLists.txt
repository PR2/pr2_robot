# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_bringup)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rostest)

# Tests that PR2 diagnostic analyzers configuration loads
add_rostest(test/pr2_analyzers_load_test.launch)

catkin_package()

# install
install(DIRECTORY config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY scripts/
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
   pr2.launch
   pr2_recalibrate.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
