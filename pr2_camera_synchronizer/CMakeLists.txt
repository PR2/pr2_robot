cmake_minimum_required(VERSION 2.8.12)
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()
project(pr2_camera_synchronizer)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure rostest)

if(CATKIN_ENABLE_TESTING)
	#DFS: removed buggy test which was causing the build to time out
   #catkin_add_nosetests(test/test_classes.py)
	#add_rostest(test/test_consistency.test)
endif()

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/CameraSynchronizer.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure)

file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" 
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*" 
   "${CMAKE_CURRENT_SOURCE_DIR}/nodes/*")

install(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
   projector_auto.launch
   projector_off.launch
   projector_on.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
