# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(imu_monitor)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY scripts/
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
