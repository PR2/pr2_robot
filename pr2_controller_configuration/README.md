This package contains YAML files that are used to configure the default
controllers and the calibration controllers on the pr2. The package also
contains a [roslaunch](https://index.ros.org/p/roslaunch) file that
configures, loads and starts the default controllers via
[pr2_controller_manager](https://index.ros.org/p/pr2_controller_manager).
The default controllers are:

- base:
- [base_controller](https://index.ros.org/p/pr2_mechanism_controllers)
- [base_odometry](https://index.ros.org/p/pr2_mechanism_controllers)
- arms:
- [r_arm_controller](https://index.ros.org/p/robot_mechanism_controllers)
- [l_arm_controller](https://index.ros.org/p/robot_mechanism_controllers)
- [r_gripper_controller](https://index.ros.org/p/pr2_mechanism_controllers)
- [l_gripper_controller](https://index.ros.org/p/pr2_mechanism_controllers)
- head:
- [head_controller](https://index.ros.org/p/robot_mechanism_controllers)
- [laser_tilt_controller](https://index.ros.org/p/pr2_mechanism_controllers)
- torso
- [torso_controller](https://index.ros.org/p/robot_mechanism_controllers)

These files are used in places such as
[pr2_bringup](https://index.ros.org/p/pr2_bringup) to instantiate the
default PR2 controllers.

For [pr2_simulator](https://index.ros.org/p/pr2_simulator), a clone of
this package with separate configuration is in
[pr2_controller_configuration_gazebo](https://index.ros.org/p/pr2_controller_configuration_gazebo).
