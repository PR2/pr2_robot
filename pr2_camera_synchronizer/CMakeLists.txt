# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_camera_synchronizer)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure rostest)

catkin_add_nosetests(test/test_classes.py)
add_rostest(test/test_consistency.test)

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
