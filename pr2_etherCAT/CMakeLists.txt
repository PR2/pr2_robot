# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_etherCAT)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
   roscpp 
   std_srvs 
   std_msgs
   pr2_controller_manager 
   ethercat_hardware 
   realtime_tools 
   diagnostic_updater)

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(pr2_etherCAT src/main.cpp)
target_link_libraries(pr2_etherCAT rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pr2_etherCAT ${catkin_EXPORTED_TARGETS})

install(TARGETS pr2_etherCAT
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY scripts/
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
