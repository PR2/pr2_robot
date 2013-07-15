# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_computer_monitor)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_add_nosetests(test/parse_test.py)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(network_detector src/network_detector.cpp)
target_link_libraries(network_detector ${catkin_LIBRARIES})

catkin_package()

install(DIRECTORY demo
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" 
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
install(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS network_detector
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_python_setup()
