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

add_executable(pr2_ethercat src/main.cpp)
target_link_libraries(pr2_ethercat rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pr2_ethercat ${catkin_EXPORTED_TARGETS})

# Set pr2_ethercat with RPATH built in so that we can install it suid
set_target_properties(pr2_ethercat PROPERTIES SKIP_BUILD_RPATH FALSE)

# Use the RPATH even in build space.
set_target_properties(pr2_ethercat PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set_target_properties(pr2_ethercat PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS pr2_ethercat
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} PERMISSIONS OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE SETUID)

file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" 
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
install(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
