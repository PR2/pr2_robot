cmake_minimum_required(VERSION 2.8.12)
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()
project(pr2_run_stop_auto_restart)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
   roscpp 
   pr2_msgs 
   pr2_power_board 
   std_srvs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(run_stop_auto_restart src/run_stop_auto_restart.cpp)
target_link_libraries(run_stop_auto_restart ${catkin_LIBRARIES})
add_dependencies(run_stop_auto_restart ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS run_stop_auto_restart
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
