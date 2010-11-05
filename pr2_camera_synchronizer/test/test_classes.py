#!/usr/bin/env python
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

PKG = "pr2_camera_synchronizer"
import roslib; roslib.load_manifest(PKG)
from pr2_camera_synchronizer.synchronizer_classes import *
import unittest
import pr2_camera_synchronizer.cfg.CameraSynchronizerConfig as Config
                             
DIGITS = 16

def issorted(l):
    copy = list(l)
    copy.sort()
    return copy == l

def interlace(l1, l2):
    l = zip(l1, l2)
    return list(sum(l, ()))

class TestProjector(unittest.TestCase):
    def runCase(self, rate, length, shift, out_repeat_period, out_proj_end, out_alt_end, out_noproj_end, level = lvl_projector):
        config = { 
                param_proj_rate : rate,  
                "projector_pulse_length" : length,
                "projector_pulse_shift" : shift,
                "projector_tweak" : 0,
                "projector_mode" : Config.CameraSynchronizer_ProjectorOn,
                }
        proj = Projector()
        proj.process_update(config, level)

        if level & lvl_projector != 0:
            self.assert_(math.fmod(config["projector_rate"], ETHERCAT_INTERVAL)) # Integer multiples of ethercat rate
            self.assert_(config["projector_pulse_length"] >= length, "Pulse length %f"%config["projector_pulse_length"]) # Never shorten the pulse length
            self.assertEqual(config["projector_pulse_length"], proj.pulse_length) # Pulse lengths are consistent
            self.assert_(math.fmod(proj.pulse_length, ETHERCAT_INTERVAL) == 0, "Pulse length %f"%proj.pulse_length) # Integer multiples of ethercat rate
            self.assertEqual(len(proj.pulse_ends), 4) # Right number of pulses 
            self.assertEqual(len(proj.pulse_starts), 4) # Right number of pulses
            self.assert_(issorted(interlace(proj.pulse_starts, proj.pulse_ends))) # Starts and stops are in right order
            self.assertEqual(4. / proj.repeat_period, config["projector_rate"]) # Base period and repeat period are consistent.

        self.assertEqual(out_repeat_period, proj.repeat_period) 
        self.assertEqual(out_proj_end, proj.proj_end_offset)
        self.assertEqual(out_noproj_end, proj.noproj_end_offset)
        self.assertAlmostEqual(out_alt_end, proj.alt_end_offset, DIGITS)

        return proj, config

    def testBasicFunctionality(self):
        proj,config = self.runCase(30, 0.002, 0,   0.132, 0.003, 0.036, 0.032)

class TestCamera(unittest.TestCase):
  def runCase(self, proj_rate, proj_length, proj_shift, rate, trig_mode, out_period, out_ext_trig, out_imager_period, out_end_offset, level = 1, reset_camera = False):
      paramnames = dict((name,name) for name in camera_parameters)
      config = {
              param_proj_rate : proj_rate,  
              "projector_pulse_length" : proj_length,
              "projector_pulse_shift" : proj_shift,
              "projector_tweak" : 0,
              "projector_mode" : Config.CameraSynchronizer_ProjectorOn,
              param_rate : rate,
              param_trig_mode : trig_mode,
              "camera_reset" : reset_camera,
              }

      proj = Projector()
      proj.process_update(config, lvl_projector)
      camera = Camera("test_cam", proj, 1, **paramnames)
      camera.process_update(config, level)

      if level & 1 == 0:
          self.assert_(math.fmod(camera.period, ETHERCAT_INTERVAL) == 0) # Integer multiples of ethercat rate
          self.assertEqual(config[param_rate], 1. / camera.period) # Rate was correctly set in config
      
      self.assert_(out_period > 0, "Period %f"%out_period) # Non-zero period
      self.assertAlmostEqual(camera.period, out_period, DIGITS) # Correct camera period
      self.assertEqual(camera.ext_trig, out_ext_trig) # Ext trig when expected
      self.assert_(out_imager_period > 0, "Imager period %f"%out_imager_period) # Non-zero imager period
      self.assertAlmostEqual(camera.imager_period, out_imager_period, DIGITS) # Correct imager period
      self.assertAlmostEqual(camera.end_offset, out_end_offset, DIGITS) # Correct end offset

      return camera, proj, config

  def testBasicAlwaysProj(self): 
      self.runCase(60, 0.001, 0,   30, Config.CameraSynchronizer_WithProjector,      0.034,  True,  0.033, 0.002) # Always proj
  def testBasicAlternateProj(self):
      self.runCase(60, 0.001, 0,   30, Config.CameraSynchronizer_AlternateProjector, 0.034,  True,  0.033, 0.019) # Alternate proj
  def testBasicNoProj(self):
      self.runCase(60, 0.001, 0,   30, Config.CameraSynchronizer_WithoutProjector,   0.034,  True,  0.033, 0.016) # No proj
  def testBasicFreeRun(self):
      self.runCase(60, 0.001, 0,   30, Config.CameraSynchronizer_InternalTrigger,   1/30.0, False, 1/30.0,    -1) # Freerun

if __name__ == '__main__':
    import rostest
    import rospy
    import time

    # Not starting a node, so we can't use rospy's time functions.
    rospy.get_time = time.time
    
    rostest.rosrun(PKG, 'test_projector', TestProjector)
    rostest.rosrun(PKG, 'test_camera', TestCamera)
    killAsynchronousUpdaters()
