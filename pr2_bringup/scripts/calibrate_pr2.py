#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Author: Kevin Watts

# Calibrates the PR-2 in a safe sequence

from __future__ import with_statement

import roslib
import copy
import yaml
import threading
import sys, os
import time
from time import sleep
import getopt

# Loads interface with the robot.
roslib.load_manifest('pr2_bringup')
import rospy
from std_msgs.msg import *
from pr2_mechanism_msgs.srv import LoadController, UnloadController, SwitchController, SwitchControllerRequest
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from sensor_msgs.msg import *


load_controller = rospy.ServiceProxy('pr2_controller_manager/load_controller', LoadController)
unload_controller = rospy.ServiceProxy('pr2_controller_manager/unload_controller', UnloadController)
switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
up_controllers = []
hold_position = {'r_shoulder_pan': -0.7, 'l_shoulder_pan': 0.7, 'r_elbow_flex': -2.0, 
                 'l_elbow_flex': -2.0, 'r_upper_arm_roll': 0.0, 'l_upper_arm_roll': 0.0, 
                 'r_shoulder_lift': 1.0, 'l_shoulder_lift': 1.0}
services = {}
controllers = {}
status = {}
publishers = []
calibration_params_namespace = "calibration_controllers"


def joint_states_cb(msg):
    global last_joint_states
    last_joint_states = msg
rospy.Subscriber('joint_states', JointState, joint_states_cb)
    
def motor_state_cb(msg):
    global motors_halted
    motors_halted = msg.data
    rospy.logdebug("motors halted = %d"%motors_halted)
rospy.Subscriber('pr2_etherCAT/motors_halted', Bool, motor_state_cb)


def calibrate(joints):
    if type(joints) is not list:
        joints = [joints]

    # Starts the calibration controllers
    rospy.logdebug("Calibrating joints %s"%joints)
    switch_controller([controllers[j] for j in joints], [], SwitchControllerRequest.BEST_EFFORT)

    # Waits for the calibration controllers to complete
    start_time = rospy.Time.now()
    delay = rospy.Duration(20.0)
    waiting_for = joints[:]
    rospy.logdebug("waiting for calibration")
    while waiting_for and not rospy.is_shutdown():
        remove = []
        for j in waiting_for:
            try:
                services[j]()
                rospy.loginfo("Finished calibrating joint %s"%j) 
                remove.append(j)
            except:
                if motors_halted:
                    rospy.logwarn('Calibration is on hold because motors are halted. Enable the run-stop')
                    start_time = rospy.Time.now()
                    rospy.sleep(1.0)
                elif rospy.Time.now() > start_time + delay:
                    rospy.logwarn("Joint %s is taking a long time to calibrate. It might be stuck and need some human help"%j)
                    rospy.sleep(1.0)
        for r in remove:
            waiting_for.remove(r)
        rospy.sleep(0.05)

    # Stops the calibration controllers
    switch_controller([], [controllers[j] for j in joints], SwitchControllerRequest.BEST_EFFORT)


def hold(joint, command):
    controller = "%s/hold/%s_position_controller" % (calibration_params_namespace, joint)
    if controller not in up_controllers:
        try:
            # loads the holding controllers
            resp = load_controller(controller)
            if resp.ok == 0:
                rospy.logerr('Failed to start controller %s'%controller)
                return
            up_controllers.append(controller)

            # Starts the holding controllers
            resp = switch_controller([controller], [], SwitchControllerRequest.BEST_EFFORT)
            if resp.ok == 0:
                rospy.logerr('Failed to start controller %s'%controller)
                return
        except Exception, ex:
            rospy.logerr("Failed to load holding controller %s: %s" % (controller, str(ex)))

    pub = rospy.Publisher("%s/command" % controller, Float64, latch=True)
    pub.publish(Float64(command))
    return pub


def flatten(l, ltypes=(list, tuple)):
    ltype = type(l)
    l = list(l)
    i = 0
    while i < len(l):
        while isinstance(l[i], ltypes):
            if not l[i]:
                l.pop(i)
                i -= 1
                break
            else:
                l[i:i + 1] = l[i]
        i += 1
    return ltype(l)


def is_calibrated_group(joints_group):
    all_joints = flatten(joints_group)
    
    # spawn calibration controllers for this group
    for j in all_joints:
        controllers[j] =  calibration_params_namespace+"/calibrate/cal_%s" % j
        rospy.logdebug("Launching: %s" %controllers[j])
        resp = load_controller(controllers[j])
        if resp.ok == 0:
            rospy.logerr("Failed: %s" %controllers[j])
            return False
        up_controllers.append(controllers[j])

    # create service client to each controller
    for j in all_joints:
        service_name = '%s/is_calibrated'%controllers[j]
        rospy.logdebug("Waiting for service: %s" %service_name)
        rospy.wait_for_service(service_name)
        services[j] = rospy.ServiceProxy(service_name, Empty)

    # check if all joints are calibrated
    rospy.logdebug('Checking which joints need calibration')
    for j in all_joints:
        try:
            services[j]()
            rospy.logdebug("joint %s is already calibrated"%j)
        except:
            rospy.logdebug("joint %s needs to be calibrated"%j)
            rospy.loginfo("These joints will get calibrated: %s"%all_joints)
            return False

    rospy.loginfo("These joints are already calibrated: %s"%all_joints)
    return True



def calibrate_group(joints_group):
    # calibrate all joints in group
    for joints  in joints_group:
        status["active"] = joints
        publish_status(status, status_pub)
        calibrate(joints)
        for j in joints:
            if j in hold_position:
                publishers.append( hold(j, hold_position[j]) )
        status["done"].extend(status["active"])




def publish_status(status, pub):
  str = "====\n"
  str += "Calibrating: %s\n"%", ".join(status["active"])
  str += "Calibrated: %s\n"%", ".join(status["done"])
  pub.publish(str)



def main():
    rospy.init_node('calibration', anonymous=True, disable_signals=True)
    calibration_start_time = rospy.Time.now()

    imustatus = True
    joints_status = False
    try:
        try:
            pub_calibrated = rospy.Publisher('calibrated', Bool, latch=True)
            pub_calibrated.publish(False)
            global status_pub
            status_pub = rospy.Publisher('calibration_status', String)
            status["active"] = []
            status["done"] = []

            allowed_flags = ['alpha-casters', 'alpha-head', 'alpha2b-head', 'arms=']
            opts, args = getopt.gnu_getopt(rospy.myargv(), 'h', allowed_flags)

            casters = ['caster_fl', 'caster_fr', 'caster_bl', 'caster_br']
            head = ['head_pan', 'head_tilt']
            arms = 'auto'

            for o, a in opts:
                if o == '-h':
                    rospy.loginfo("Flags:", ' '.join(['--'+f for f in allowed_flags]))
                    sys.exit(0)
                elif o == '--alpha-casters':
                    casters = ['caster_fl_alpha2', 'caster_fr_alpha2',
                               'caster_bl_alpha2', 'caster_br_alpha2']
                elif o == '--alpha-head':
                    head = ['head_pan_alpha2', 'head_tilt']
                elif o == '--alpha2b-head':
                    head = ['head_pan_alpha2', 'head_tilt_alpha2b']
                elif o == '--arms':
                    arms = a

            if arms not in ['both', 'none', 'left', 'right', 'auto']:
                print 'Arms must be "both", "none", "left", "right", or "auto"'
                sys.exit(1)

            pr2_controller_configuration_dir = roslib.packages.get_pkg_dir('pr2_controller_configuration')
            calibration_yaml = '%s/pr2_calibration_controllers.yaml' % pr2_controller_configuration_dir
            hold_yaml = '%s/pr2_joint_position_controllers.yaml' % pr2_controller_configuration_dir

            if len(args) < 3:
               rospy.loginfo("No yaml files specified for calibration and holding controllers, using defaults")
            else:
               calibration_yaml = args[1]
               hold_yaml  = args[2]
            rospy.wait_for_service('pr2_controller_manager/load_controller')
            rospy.wait_for_service('pr2_controller_manager/switch_controller')
            rospy.wait_for_service('pr2_controller_manager/unload_controller')

            # Determines whether to calibrate the arms based on which joints exist.
            if arms == 'auto':
                started_waiting = rospy.get_rostime()
                global last_joint_states
                while rospy.get_rostime() <= started_waiting + rospy.Duration(5.0):
                    if last_joint_states:
                        break
                js = last_joint_states
                if not js:
                    arms = 'both'
                else:
                    if 'r_shoulder_pan_joint' in js.name and 'l_shoulder_pan_joint' in js.name:
                        arms = 'both'
                    elif 'r_shoulder_pan_joint' in js.name:
                        arms = 'right'
                    elif 'l_shoulder_pan_joint' in js.name:
                        arms = 'left'
                    else:
                        arms = 'none'
                rospy.logout("Arm selection was set to \"auto\".  Chose to calibrate using \"%s\"" % arms)

            # define calibration groups
            r_arm_group = [['r_shoulder_pan'], ['r_elbow_flex'], ['r_upper_arm_roll'], ['r_shoulder_lift'], ['r_forearm_roll', 'r_wrist']]
            l_arm_group = [['l_shoulder_pan'], ['l_elbow_flex'], ['l_upper_arm_roll'], ['l_shoulder_lift'], ['l_forearm_roll', 'l_wrist']]
            b_arm_group = [['r_shoulder_pan', 'l_shoulder_pan'], ['r_elbow_flex', 'l_elbow_flex'],
                           ['r_upper_arm_roll', 'l_upper_arm_roll'], ['r_shoulder_lift', 'l_shoulder_lift'],
                           ['r_forearm_roll', 'r_wrist', 'l_forearm_roll', 'l_wrist']]
            torso_group = [['torso_lift']]
            gripper = []
            if arms in ['right', 'both']:
                gripper.append('r_gripper')
            if arms in ['left', 'both']:
                gripper.append('l_gripper')
            gripper_group = [gripper]
            head_group = [head, 'laser_tilt']
            casters_group = [casters]

            # load calibration controllers configuration
            rospy.loginfo("Loading controller configuration on parameter server...")
            rospy.set_param(calibration_params_namespace+"/calibrate", yaml.load(open(calibration_yaml)))
            rospy.set_param(calibration_params_namespace+"/hold", yaml.load(open(hold_yaml)))

            # check which groups need calibration
            arm_group_calibrated = False
            if arms == 'both':
                arm_group_calibrated = is_calibrated_group(b_arm_group)
            elif arms == 'right':
                arm_group_calibrated = is_calibrated_group(r_arm_group)
            elif arms == 'left':
                arm_group_calibrated = is_calibrated_group(l_arm_group)
            torso_group_calibrated = is_calibrated_group(torso_group)
            gripper_group_calibrated = is_calibrated_group(gripper_group)
            casters_group_calibrated = is_calibrated_group(casters_group)
            head_group_calibrated = is_calibrated_group(head_group)

            # calibrate imu and torso
            if not torso_group_calibrated:
                rospy.loginfo('Calibrating imu')
                status["active"] = ["imu"]
                imu_calibrate_srv_name = '/torso_lift_imu/calibrate'
                rospy.wait_for_service(imu_calibrate_srv_name)  
                imu_calibrate_srv = rospy.ServiceProxy(imu_calibrate_srv_name, Empty)
                try:
                    imu_calibrate_srv()
                    imustatus = True
                    # only calibrate torso when imu calibration succeeds
                    calibrate_group(torso_group)
                except:
                    imustatus = False
                status["done"].extend(status["active"])
                rospy.loginfo('Calibrating imu finished')
            else:
                rospy.loginfo('Not calibrating imu')

            # calibrate arms
            if not arm_group_calibrated:
                publishers.append( hold('torso_lift', 0.25) )
                rospy.sleep(5.0)
                rospy.loginfo('Moving up spine to allow arms to calibrate')
                if arms == 'both':
                    calibrate_group(b_arm_group)
                elif arms == 'right':
                    calibrate_group(r_arm_group)
                elif arms == 'left':
                    calibrate_group(l_arm_group)
                rospy.loginfo('Moving down spine after arm calibration')
                publishers.append( hold('torso_lift', 0.01) )
                rospy.sleep(20.0)

            # calibrate grippers
            if not gripper_group_calibrated:
                calibrate_group(gripper_group)

            # calibrate head
            if not head_group_calibrated:
                calibrate_group(head_group)

            # calibrate casters
            if not casters_group_calibrated:
                calibrate_group(casters_group)

            joints_status = True
            status_pub.publish("CALIBRATION COMPLETE")



        finally:
            rospy.loginfo("Bringing down calibration node")

            # stop/unload all controllers
            try:
                switch_controller([], up_controllers, SwitchControllerRequest.BEST_EFFORT)
                # Unload all controllers
                for name in up_controllers:
                    resp_unload = unload_controller(name)
                    if (resp_unload == 0):
                        rospy.logerr("Failed to unload controller %s" % name)
            except rospy.ServiceException:
                rospy.logerr("Call to switch_controller failed")
            except KeyboardInterrupt:
                raise

            # Unregister all holding publishers
            for p in publishers:
                p.unregister()

            # clears controller parameters
            rospy.set_param(calibration_params_namespace, "")

            if not imustatus and not joints_status:
                rospy.logerr("Both mechanism and IMU calibration failed")
            elif not joints_status:
                rospy.logerr("IMU calibration complete, but mechanism calibration failed")
            elif not imustatus:
                rospy.logerr("Mechanism calibration complete, but IMU calibration failed.")
            else:
                rospy.loginfo('Calibration completed in %f sec' %(rospy.Time.now() - calibration_start_time).to_sec())

            if joints_status:
                pub_calibrated.publish(True)

        # Spin
        while True:
            time.sleep(1e6)
    except KeyboardInterrupt:
        pass
    finally:
        rospy.signal_shutdown("interrupted")
        

if __name__ == '__main__': main()
