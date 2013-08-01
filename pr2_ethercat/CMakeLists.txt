# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_ethercat)

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

add_definitions(-O3)

add_executable(pr2_ethercat src/main.cpp)
target_link_libraries(pr2_ethercat rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pr2_ethercat ${catkin_EXPORTED_TARGETS})
pr2_enable_rpath(pr2_ethercat)

install(TARGETS pr2_ethercat
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" 
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
install(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
