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
from sensor_msgs.msg import *


load_controller = rospy.ServiceProxy('pr2_controller_manager/load_controller', LoadController)
unload_controller = rospy.ServiceProxy('pr2_controller_manager/unload_controller', UnloadController)
switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)

controllers_up = []
calibration_params_namespace = "calibration_controllers"


last_joint_states = None
def joint_states_cb(msg):
    global last_joint_states
    last_joint_states = msg
rospy.Subscriber('/joint_states', JointState, joint_states_cb)
    

def calibrate(joints):
    if type(joints) is not list:
        joints = [joints]

    controllers = [calibration_params_namespace+"/calibrate/cal_%s" % j for j in joints]
    launched = []
    try:
        # Launches the calibration controllers
        for c in controllers:
            resp = load_controller(c)
            if resp.ok == 0:
                rospy.logerr("Failed: %s" % c)
            else:
                launched.append(c)
        rospy.loginfo("Launched: %s" % ', '.join(launched))

        # Starts the launched controllers
        switch_controller(launched, [], SwitchControllerRequest.BEST_EFFORT)

        # Waits for the calibration controllers to complete
        waiting_for = launched[:]
        launch_time = rospy.Time.now()
        def calibrated(msg, name): 
            try:
                if name in waiting_for:
                    waiting_for.remove(name)
            except:
                pass
        subscribers = []
        waiting_for_copy = waiting_for[:]
        for name in waiting_for_copy:
            subscribers.append(rospy.Subscriber("%s/calibrated" % name, Empty, calibrated, name))

        # Waits until all the controllers have calibrated
        count = 0
        while waiting_for and not rospy.is_shutdown():
            count += 1
            if count % 20 == 0:
                rospy.logdebug("Waiting for: %s" % ', '.join(waiting_for))
                #wait 20 seconds for calibration to complete, if this doesn't happen, warn the user
                if (rospy.Time.now() - launch_time) > rospy.Duration(15.0):
                  rospy.logwarn("Calibration on the %s joint(s) is taking longer than expected, something may be stuck and the robot may require human help." % ', '.join(waiting_for))
            sleep(0.1)
    finally:
        # Try to unload controllers several times
        # Make sure they're dead.
        for name in launched:
            try:
                resp_stop = switch_controller([], [name], SwitchControllerRequest.STRICT)
                if (resp_stop == 0):
                    rospy.logerr("Failed to stop controller %s" % name)
                resp_unload = unload_controller(name)
                if (resp_unload == 0):
                    rospy.logerr("Failed to unload controller %s" % name)
            except Exception, ex:
                rospy.logerr("Failed to stop/unload controller %s" % name)

        # unsubscribe from calibrate topics
        for s in subscribers:
            s.unregister()

def hold(joint, command):
    controller = "%s/hold/%s_position_controller" % (calibration_params_namespace, joint)
    if controller not in controllers_up:
        for i in range(3):
            try:
                resp = load_controller(controller)
                if resp.ok != 0:
                    controllers_up.append(controller)

                    # Starts the launched controllers
                    switch_controller([controller], [], SwitchControllerRequest.BEST_EFFORT)

                    break
                else:
                    rospy.logerr("Error loading %s" % controller)
            except Exception, ex:
                rospy.logerr("Failed to load holding controller %s on try %d: %s" % (controller, i+1, str(ex)))

    pub = rospy.Publisher("%s/command" % controller, Float64, latch=True)
    pub.publish(Float64(command))
    return pub

def calibrate_imu():
    class is_calibrated_helper:
        def __init__(self):
            self.is_calibrated = False
            self.cond = threading.Condition()

        def callback(self, msg):
            if msg.data:
                with self.cond:
                    self.is_calibrated = True
                    self.cond.notify()

        def wait_for_calibrated(self, topic, timeout):
            self.sub = rospy.Subscriber(topic,Bool,self.callback)
            try:
                with self.cond:
                    if not self.is_calibrated:
                        self.cond.wait(timeout)
                return self.is_calibrated
            finally:
                self.sub.unregister()

    rospy.loginfo("Waiting up to 20s for IMU calibration to complete.")
    helper = is_calibrated_helper()
    if not helper.wait_for_calibrated("imu_data/is_calibrated", 20):
        rospy.logerr("IMU took too long to calibrate.")
        return False
    return True

def publish_status(status, pub):
  str = "====\n"
  str += "Calibrating: %s\n"%", ".join(status["active"])
  str += "Calibrated: %s\n"%", ".join(status["done"])
  pub.publish(str)

def main():
    publishers = []
    imustatus = False
    joints_status = False
    try:
        try:
            pub_calibrated = rospy.Publisher('calibrated', Bool, latch=True)
            rospy.init_node('calibration', anonymous=True, disable_signals=True)
            pub_calibrated.publish(False)
            status_pub = rospy.Publisher('calibration_status', String)
            status = {}
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

            # Calibrate all joints sequentially
            # Hold joints after they're calibrated.

            status["active"] = ["imu"]

            imustatus = calibrate_imu()
            if not imustatus:
                rospy.logerr("IMU Calibration failed.")


            # load calibration controllers configuration
            rospy.loginfo("Loading controller configuration on parameter server...")
            rospy.set_param(calibration_params_namespace+"/calibrate", yaml.load(open(calibration_yaml)))
            rospy.set_param(calibration_params_namespace+"/hold", yaml.load(open(hold_yaml)))

            status["done"].extend(status["active"])
            status["active"] = ['torso_lift']
            publish_status(status, status_pub)

            calibrate('torso_lift')
            publishers.append( hold('torso_lift', 0.08) )
            sleep(0.5)

            status["done"].extend(status["active"])
            publish_status(status, status_pub)
            if arms in ['both', 'right']:
                status["active"] = ['r_shoulder_pan']
                publish_status(status, status_pub)

                calibrate('r_shoulder_pan')
                publishers.append( hold('r_shoulder_pan', -0.7) )

                status["done"].extend(status["active"])
            if arms in ['both', 'left']:
                status["active"] = ['l_shoulder_pan']
                publish_status(status, status_pub)

                calibrate('l_shoulder_pan')
                publishers.append( hold('l_shoulder_pan', 0.7) )

                status["done"].extend(status["active"])

            if arms == 'both':
                status["active"] = ['r_elbow_flex', 'l_elbow_flex']
                publish_status(status, status_pub)

                calibrate(['r_elbow_flex', 'l_elbow_flex'])
                publishers.append( hold('r_elbow_flex', -2.0) )
                publishers.append( hold('l_elbow_flex', -2.0) )

                status["done"].extend(status["active"])
                status["active"] = ['r_upper_arm_roll', 'l_upper_arm_roll']
                publish_status(status, status_pub)

                calibrate(['r_upper_arm_roll', 'l_upper_arm_roll'])
                publishers.append( hold('r_upper_arm_roll', 0.0) )
                publishers.append( hold('l_upper_arm_roll', 0.0) )

                status["done"].extend(status["active"])
                status["active"] = ['r_shoulder_lift', 'l_shoulder_lift']
                publish_status(status, status_pub)

                calibrate(['r_shoulder_lift', 'l_shoulder_lift'])
                publishers.append( hold('r_shoulder_lift', 1.0) )
                publishers.append( hold('l_shoulder_lift', 1.0) )
            elif arms == 'right':
                status["active"] = ['r_elbow_flex']
                publish_status(status, status_pub)
                calibrate(['r_elbow_flex'])
                publishers.append( hold('r_elbow_flex', -2.0) )

                status["done"].extend(status["active"])
                status["active"] = ['r_upper_arm_roll']
                publish_status(status, status_pub)

                calibrate(['r_upper_arm_roll'])
                publishers.append( hold('r_upper_arm_roll', 0.0) )

                status["done"].extend(status["active"])
                status["active"] = ['r_shoulder_lift']
                publish_status(status, status_pub)

                calibrate(['r_shoulder_lift'])
                publishers.append( hold('r_shoulder_lift', 1.0) )
            elif arms == 'left':
                status["active"] = ['l_elbow_flex']
                publish_status(status, status_pub)

                calibrate(['l_elbow_flex'])
                publishers.append( hold('l_elbow_flex', -2.0) )

                status["done"].extend(status["active"])
                status["active"] = ['l_upper_arm_roll']
                publish_status(status, status_pub)

                calibrate(['l_upper_arm_roll'])
                publishers.append( hold('l_upper_arm_roll', 0.0) )

                status["done"].extend(status["active"])
                status["active"] = ['l_shoulder_lift']
                publish_status(status, status_pub)

                calibrate(['l_shoulder_lift'])
                publishers.append( hold('l_shoulder_lift', 1.0) )
                

            publish_status(status, status_pub)
            publishers.append( hold('torso_lift', 0.0) )
            sleep(1.0)

            # Everything else

            common = ['laser_tilt']
            if arms in ['right', 'both']:
                common.extend(['r_forearm_roll', 'r_wrist', 'r_gripper'])
            if arms in ['left', 'both']:
                common.extend(['l_forearm_roll', 'l_wrist', 'l_gripper'])

            status["done"].extend(status["active"])
            status["active"] = common[:]
            publish_status(status, status_pub)

            calibrate(common + head + casters)

            joints_status = True
            status_pub.publish("CALIBRATION COMPLETE")

        finally:
            print "Bringing down calibration things"
            # Unload all holding controllers
            for name in controllers_up:
                try:
                    resp_stop = switch_controller([], [name], SwitchControllerRequest.STRICT)
                    if (resp_stop == 0):
                        rospy.logerr("Failed to stop controller %s" % name)
                    resp_unload = unload_controller(name)
                    if (resp_unload == 0):
                        rospy.logerr("Failed to unload controller %s" % name)
                except Exception, ex:
                    rospy.logerr("Failed to stop/unload controller %s" % name)

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
                rospy.logout("Calibration complete")

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
