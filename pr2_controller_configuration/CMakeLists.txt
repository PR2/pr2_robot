cmake_minimum_required(VERSION 2.8.12)
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()
project(pr2_controller_configuration)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

install(FILES
   pr2_arm_controllers.yaml
   pr2_base_controller2.yaml
   pr2_base_controller.yaml
   pr2_calibration_controllers.yaml
   pr2_gripper_controllers.yaml
   pr2_head_controller.yaml
   pr2_joint_effort_controllers.yaml
   pr2_joint_position_controllers.yaml
   pr2_joint_velocity_controllers.yaml
   pr2_laser_tilt_controller.yaml
   pr2_odometry.yaml
   pr2_torso_controller.yaml
   pr2_default_controllers.launch
   pr2_jt_controllers.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


# roslaunch checks
roslaunch_add_file_check(pr2_default_controllers.launch)
#roslaunch_add_file_check(pr2_jt_controllers.launch)
