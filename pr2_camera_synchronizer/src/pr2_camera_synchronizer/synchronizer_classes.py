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

from __future__ import with_statement
import roslib; roslib.load_manifest('pr2_camera_synchronizer')
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from ethercat_trigger_controllers.srv import SetMultiWaveform as SetMultiWaveform
from ethercat_trigger_controllers.srv import SetMultiWaveformRequest as SetMultiWaveformRequest
from ethercat_trigger_controllers.msg import MultiWaveform as MultiWaveform
from ethercat_trigger_controllers.msg import MultiWaveformTransition as MultiWaveformTransition
import pr2_camera_synchronizer.cfg.CameraSynchronizerConfig as Config
import wge100_camera.cfg.WGE100CameraConfig as WGEConfig

import rospy
import time
import math
import threading
import signal

from pr2_camera_synchronizer.cfg import CameraSynchronizerConfig as ConfigType
from pr2_camera_synchronizer.levels import *
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray

ETHERCAT_INTERVAL = 0.001

param_rate = "rate"
param_trig_mode = "trig_mode"
camera_parameters = [ param_rate, param_trig_mode ]

param_proj_rate = "projector_rate"

def roundToEthercat(val):
    return ETHERCAT_INTERVAL * round(val / ETHERCAT_INTERVAL)

asynchronous_updaters = []
def killAsynchronousUpdaters():
    for updater in asynchronous_updaters:
        updater.kill()
class AsynchronousUpdater(threading.Thread):
    def __init__(self, f, name):
        threading.Thread.__init__(self)
        self.name = name
        self.f = f
        self.allargs = None
        self.cv = threading.Condition()
        asynchronous_updaters.append(self)
        self.exiting = False
        self.idle = True
        self.start()

    def update(self, *args, **nargs):
        with self.cv:
            self.allargs = (args, nargs)
            self.cv.notify()
            #print "update", self.allargs

    def run(self):
        #print "run"
        while True:
            #print "Starting loop on", self.name
            with self.cv:
                if self.exiting:
                    break
                if self.allargs == None:
                  #print "start wait"
                   self.idle = True
                   self.cv.wait()
                   self.idle_end_time = rospy.get_time()
                   self.idle = False
                  #print "end wait"
                allargs = self.allargs
                self.allargs = None
            if allargs != None:
                try:
                    self.f(*allargs[0], **allargs[1])
                except Exception, e:
                    rospy.logerr("AsynchronousUpdater failed with exception: %s"%str(e))
                    pass

    def kill(self):
        #print "kill"
        #print "Killing", self.name
        with self.cv:
            self.exiting = True
            self.allargs = None
            self.cv.notify()

    def getStatus(self): # For diagnostics
        if self.idle:
            return self.name, 0
        interval = rospy.get_time() - self.idle_end_time
        return self.name, interval

class MultiTriggerController:
  def __init__(self, name):
    self.period = 0
    self.zero_offset = 0
    self.clear_waveform()
    self.name = name
    self.async = AsynchronousUpdater(self.async_update, "Controller "+name)
    self.service = None
    self.transitions = []

  def clear_waveform(self):
    self.transitions = []
                                                    
  def add_sample(self, time, value, topic):
    time = roundToEthercat(time) + 0.5 * ETHERCAT_INTERVAL
    self.transitions.append(MultiWaveformTransition(time, value, topic))
  
  def async_update(self, period, zero_offset, transitions):
      try:
          if self.service == None:
              service_name = self.name+"/set_waveform"
              rospy.wait_for_service(service_name)
              self.service = rospy.ServiceProxy(service_name, SetMultiWaveform)
              #print "Service", service_name, "exists."

          #print "Trigger async_update got proxy on", self.name
          waveform = MultiWaveform(period, zero_offset, transitions)
          #print "Updating waveform ", self.name, waveform
          rslt = self.service(waveform)
          if not rslt.success:
              rospy.logerr("Error setting waveform %s: %s"%(self.name, rslt.status_message))
          #print "Done updating waveform ", self.name
      except KeyboardInterrupt: # Handle CTRL+C
          print "Aborted trigger update on", self.name

  def update(self):
      # Run the update using an Asynchronous Updater so that if something
      # locks up, the rest of the node can keep working.
      #print "Trigger update on", self.name
      self.async.update(self.period, self.zero_offset, self.transitions)

class ProsilicaInhibitTriggerController(MultiTriggerController):
    def __init__(self, name, param, true_val, false_val):
        MultiTriggerController.__init__(self, name)
        self.param = param
        self.true_val = true_val
        self.false_val = false_val

    def process_update(self, config, level):
        self.period = 1
        self.zero_offset = 0
        self.clear_waveform()
        self.add_sample(0, { True: self.true_val, False: self.false_val}[config[self.param]], '-')
        MultiTriggerController.update(self)

class ProjectorTriggerController(MultiTriggerController):
    def __init__(self, name, proj):
        MultiTriggerController.__init__(self, name)
        self.proj = proj

    def update(self):
        self.period = self.proj.repeat_period
        self.zero_offset = self.proj.zero_offset
        self.clear_waveform()
        self.high_val = 0xf
        if (self.proj.needed == False and self.proj.mode == Config.CameraSynchronizer_ProjectorAuto) or \
                self.proj.mode == Config.CameraSynchronizer_ProjectorOff:
            self.high_val = 0xe
        for i in range(0, len(self.proj.pulse_starts)):
          self.add_sample(self.proj.pulse_starts[i], self.high_val, 'on_time')
          self.add_sample(self.proj.pulse_ends[i], 0xe, 'off_time')
        MultiTriggerController.update(self)

class SingleCameraTriggerController(MultiTriggerController):
  def __init__(self, name, camera):
    MultiTriggerController.__init__(self, name)
    self.camera = camera

  def update(self):
    if self.camera.reset_cameras:
      self.camera_reset()
      return
    self.clear_waveform()
    if not self.camera.ext_trig:
      self.period = 1
      self.add_sample(0, 0, "-")
    else:
      self.period = 2 * self.camera.period
      self.zero_offset = -self.camera.imager_period 
      first_pulse_start = self.camera.end_offset
      second_pulse_start = self.camera.period + first_pulse_start
      first_frame_end = first_pulse_start + self.camera.imager_period
      extra_pulse_start = (first_pulse_start + first_frame_end) / 2  
      trigger_name = "trigger"
      self.add_sample(first_pulse_start, 1, trigger_name)
      self.add_sample((first_pulse_start + extra_pulse_start) / 2, 0, "-")
      self.add_sample(extra_pulse_start, 1, "-")
      self.add_sample((extra_pulse_start + first_frame_end) / 2, 0, "-")
      self.add_sample(second_pulse_start, 1, trigger_name)
      self.add_sample((second_pulse_start + self.period) / 2, 0, "-")
      self.camera.trig_rising = True
      self.camera.trigger_name = self.name+"/"+trigger_name
    
    #print "About to update trigger", self.name
    MultiTriggerController.update(self)

  def camera_reset(self):
    self.clear_waveform()
    self.period = 1.5
    self.add_sample(0, 0, "-")
    self.add_sample(0.1, 1, "-")
    MultiTriggerController.update(self)

class DualCameraTriggerController(SingleCameraTriggerController):
  def __init__(self, name, camera1, camera2):
    SingleCameraTriggerController.__init__(self, name, None)
    self.cameras = [camera1, camera2]

  def update(self):
    if self.cameras[0].reset_cameras or self.cameras[1].reset_cameras:
      self.camera_reset()
      return
    
    if self.cameras[0].period != self.cameras[1].period or \
       self.cameras[0].imager_period != self.cameras[1].imager_period:
       rospy.logerr("Cameras on same trigger have different period/imager_period settings.")
    
    self.clear_waveform()
    if self.cameras[0].end_offset == self.cameras[1].end_offset:
        # This works because all cameras have the same imager period.
        self.camera = self.cameras[0]
        SingleCameraTriggerController.update(self)
        self.cameras[1].trigger_name = self.cameras[0].trigger_name
        self.cameras[1].trig_rising = self.cameras[0].trig_rising
        return
    
    if self.cameras[0].end_offset > self.cameras[1].end_offset:
        # Reduction to the case where cameras[0] happens first.
        self.cameras.reverse()

    # Now we are sure that camera 1 happens strictly before camera 2.
    assert self.cameras[0].proj == self.cameras[1].proj
    assert self.cameras[0].period == self.cameras[1].period # Will fail if both stereo can be alternate or if they are updated in the wrong order.

    self.cameras[0].trig_rising = True
    self.cameras[1].trig_rising = False

    self.clear_waveform()
    self.period = 2 * self.cameras[1].period
    self.zero_offset = -self.cameras[0].imager_period 

    first_rise = self.cameras[0].end_offset
    first_fall = self.cameras[1].end_offset
    first_rise_end = first_rise + self.cameras[0].imager_period
    second_rise = self.cameras[0].end_offset + self.cameras[0].period
    second_fall = self.cameras[1].end_offset + self.cameras[1].period
    extra_rise = (2 * first_fall + first_rise_end) / 3                            
    extra_fall = (first_fall + 2 * first_rise_end) / 3
                                              
    trigger_name = [ "trigger_"+camera.name for camera in self.cameras ]
    self.add_sample(first_rise,  1, trigger_name[0])
    self.add_sample(first_fall,  0, trigger_name[1])
    self.add_sample(extra_rise,  1, "-")
    self.add_sample(extra_fall,  0, "-")
    self.add_sample(second_rise, 1, trigger_name[0])
    self.add_sample(second_fall, 0, trigger_name[1])
    for i in range(0,2):
      self.cameras[i].trigger_name = self.name+"/"+trigger_name[i]
      #print self.cameras[i].trigger_name
    
    #print "About to update trigger", self.name
    MultiTriggerController.update(self)

class Projector:
  def process_update(self, config, level):
    # Produces:
    # MEMBERS
    # proj_end_offset, alt_end_offset, noproj_end_offset
    # pulse_starts, pulse_ends
    # repeat_period
    #
    # PARAMETERS 
    # projector_rate and projector_pulse_length
    #print "Setting needed to false."
    self.needed = False # If a camera needs us, the camera will say so.

    if level & lvl_projector == 0:
      return; # The projector's configuration is unchanged.

    base_period = 1.0 / config["projector_rate"]
    self.pulse_length = config["projector_pulse_length"]
    pulse_shift = config["projector_pulse_shift"]
    self.zero_offset = roundToEthercat(config["projector_tweak"])
    self.mode = config["projector_mode"]

    base_period = roundToEthercat(base_period)
    # Minimum is guard, pulse, guard, offset_pulse
    base_period = max(base_period, 4 * ETHERCAT_INTERVAL)
    
    self.repeat_period = 4.0 * base_period

    # Pulse can't be so long that it would coalesce with next one.
    self.pulse_length = min(self.pulse_length, math.floor(base_period / ETHERCAT_INTERVAL / 2.0 - 1) * ETHERCAT_INTERVAL)
    self.pulse_length = math.ceil(self.pulse_length / ETHERCAT_INTERVAL) * ETHERCAT_INTERVAL

    min_second_start = base_period
    max_second_start = 2 * base_period - 2 * ETHERCAT_INTERVAL - 2 * self.pulse_length
    second_start = min_second_start * (1 - pulse_shift) + max_second_start * pulse_shift
    second_start = roundToEthercat(second_start)
    config["projector_pulse_shift"] = (second_start - min_second_start) / (max_second_start - min_second_start)

    self.pulse_starts = [
        0,
        second_start,
        2 * base_period,
        2 * base_period + second_start + self.pulse_length + 2 * ETHERCAT_INTERVAL
        ]
    self.pulse_ends = [ start + self.pulse_length for start in self.pulse_starts ]

    self.proj_end_offset = self.pulse_length + ETHERCAT_INTERVAL
    self.alt_end_offset = second_start + self.pulse_length + ETHERCAT_INTERVAL
    self.noproj_end_offset = second_start - ETHERCAT_INTERVAL

    self.proj_exposure_duration = self.pulse_length + ETHERCAT_INTERVAL * 1.5
    self.noproj_max_exposure = second_start - self.pulse_length - ETHERCAT_INTERVAL * 1.5

    config["projector_rate"] = 1 / base_period
    config["projector_pulse_length"] = self.pulse_length
    config["projector_tweak"] = self.zero_offset

class Camera:
  def __init__(self, node_name, proj, level, **paramnames):
    self.paramnames = paramnames
    self.name = node_name
    self.level = level
    self.proj = proj
    self.reconfigure_client = None
    self.async = AsynchronousUpdater(self.async_apply_update, "Camera "+node_name)
    self.trig_rising = True

  def param(self, config, name):
    return config[self.paramnames[name]]

  def setparam(self, config, name, value):
    config[self.paramnames[name]] = value

  # Uses the new configuration to compute new camera state.
  def process_update(self, config, level):
    # Produces:
    # MEMBERS
    # period, ext_trig, imager_period, end_offset
    #
    # PARAMETERS
    # "param_rate"
    self.trigger_name = "not_set" # Will be set by the CameraTriggerController
    
    # Took level checking out as it was causing problems with the projector
    # needed flag.
    #if self.level & level == 0: 
    #  return # This camera's configuration is unchanged.
    
    self.period = 1.0 / self.param(config, param_rate)
    projector_rate = config[param_proj_rate]
    self.reset_cameras = config["camera_reset"]
    self.ext_trig = True
    self.register_set = WGEConfig.WGE100Camera_PrimaryRegisterSet

    projector_limits_exposure = True

    trig_mode = self.param(config, param_trig_mode)
    # Internal triggering.
    if trig_mode == Config.CameraSynchronizer_InternalTrigger:
      self.ext_trig = False
      self.imager_period = self.period
      self.end_offset = -1
      projector_limits_exposure = False
    else:
      if self.proj.mode == Config.CameraSynchronizer_ProjectorOff:
          trig_mode = Config.CameraSynchronizer_IgnoreProjector

      if trig_mode == Config.CameraSynchronizer_AlternateProjector or trig_mode == Config.CameraSynchronizer_WithProjector:
          self.proj.needed = True
          #print self.name, "setting needed to true."
      #else:
          #print self.name, "not setting needed to true."
      
      # Set the period
      if trig_mode == Config.CameraSynchronizer_AlternateProjector:
        n = round(self.period / self.proj.repeat_period - 0.5)
        n = max(n, 0)
        self.period = (n + 0.5) * self.proj.repeat_period
        #print "Case 1", n
      elif trig_mode == Config.CameraSynchronizer_IgnoreProjector:
        self.period = roundToEthercat(self.period)
      else:
        n = round(2 * self.period / self.proj.repeat_period - 1)
        n = max(n, 0)
        self.period = (n + 1) * self.proj.repeat_period / 2.0
        #print "Case 2", n, self.period

      # Set the end_offset
      if trig_mode == Config.CameraSynchronizer_IgnoreProjector:
        self.end_offset = 0
        projector_limits_exposure = False
      if trig_mode == Config.CameraSynchronizer_AlternateProjector:
        self.end_offset = self.proj.alt_end_offset 
        self.register_set = WGEConfig.WGE100Camera_Auto
      elif trig_mode == Config.CameraSynchronizer_WithProjector:
        self.end_offset = self.proj.proj_end_offset
        self.register_set = WGEConfig.WGE100Camera_AlternateRegisterSet
      else:
        self.end_offset = self.proj.noproj_end_offset

      # Pick the imager period
      if trig_mode == Config.CameraSynchronizer_IgnoreProjector:
          self.imager_period = self.period - ETHERCAT_INTERVAL
      else:
          self.imager_period = self.proj.repeat_period / 2 - ETHERCAT_INTERVAL

    #print "Camera period", self.name, self.period, self.imager_period, self.proj.repeat_period
    if projector_limits_exposure:
        self.max_exposure = self.proj.noproj_max_exposure
        #print "Exposurei projector", self.max_exposure
    else:
        self.max_exposure = self.imager_period * 0.95
        #print "Exposurei imager", self.max_exposure

    self.setparam(config, param_rate, 1/self.period)
    self.setparam(config, param_trig_mode, trig_mode)

  def async_apply_update(self, reconfig_request):
      try:    
          #print "**** Start", self.name
          if self.reconfigure_client == None:
              #print "**** Making client", self.name
              self.reconfigure_client = DynamicReconfigureClient(self.name)
              #print "**** Made client", self.name

          self.reconfigure_client.update_configuration(reconfig_request)
          #print "**** Reconfigured client", self.name
          #print "Done updating camera ", self.name
      except KeyboardInterrupt: # Handle CTRL+C
          print "Aborted camera update on", self.name
      
  def apply_update(self):
      reconfig_request = {
              "ext_trig" : self.ext_trig,
              "trig_rate" : 1.0 / self.period,
              "imager_rate" : 1.0 / self.imager_period,
              "trig_timestamp_topic" : self.trigger_name,
              "rising_edge_trig" : self.trig_rising,
              "register_set" : self.register_set,
              "auto_exposure_alternate" : False,
              "exposure_alternate" : self.proj.proj_exposure_duration,
              "max_exposure" : self.max_exposure,
              }
      #print self.name, reconfig_request

      self.async.update(reconfig_request)

# Need to set:
# Global period if synchronized
# Camera
#   Frame rate
#   Trigger mode
#   Exposure alt
#   Maximum exposure alt
#   Alternate enabled
#   Alternate only
#   Trigger name
# Trigger
#   Full waveform
# Projector
#   Full waveform
#   Prosilica setting

class CameraSynchronizer:
  def __init__(self, forearm=True):
    stereo_camera_names = [ "narrow_stereo", "wide_stereo" ] # narrow must be first as it can be alternate, and hence has more period restrictions. 
    forearm_camera_names = [ "forearm_r", "forearm_l" ]
    self.camera_names = stereo_camera_names
    if forearm:
        self.camera_names = self.camera_names + forearm_camera_names
    # Parameter names are pretty symmetric. Build them up automatically.
    parameters = [ param_rate, param_trig_mode ]
    camera_parameters = dict((name, dict((suffix, name+"_"+suffix) for suffix in parameters)) for name in self.camera_names)
    # Some parameters are common to the wide and narrow cameras. Also store
    # the node names.
    for camera in stereo_camera_names:
      camera_parameters[camera][param_rate] = "stereo_rate"
      camera_parameters[camera]["node_name"] = camera+"_both"
    if forearm:
      for camera in forearm_camera_names:
        camera_parameters[camera]["node_name"] = camera[-1]+"_forearm_cam"
    for i in range(0, len(self.camera_names)):
      camera_parameters[self.camera_names[i]]["level"] = 1 << i; # This only works because of the specific levels in the .cfg file.

    self.projector = Projector()
    self.cameras = dict((name, Camera(proj = self.projector, **camera_parameters[name])) for name in self.camera_names)
    self.prosilica_inhibit = ProsilicaInhibitTriggerController('prosilica_inhibit_projector_controller', "prosilica_projector_inhibit", 0x0A, 0x00)
    
    self.controllers = [
      ProjectorTriggerController('projector_trigger', self.projector),
      DualCameraTriggerController('head_camera_trigger', *[self.cameras[name] for name in stereo_camera_names])]
    if forearm:
      self.controllers = self.controllers + [
          SingleCameraTriggerController('r_forearm_cam_trigger', self.cameras["forearm_r"]),
          SingleCameraTriggerController('l_forearm_cam_trigger', self.cameras["forearm_l"]),
        ]

    self.server = DynamicReconfigureServer(ConfigType, self.reconfigure)

  @staticmethod
  def print_threads():
    threads = threading.enumerate()
    n = 0
    for t in threads:
        if not t.isDaemon() and t != threading.currentThread():
            try: 
               print t.name
            except:
                print "Unknown thread ", t
            n = n + 1
    return n

  def kill(self):
    print "\nWaiting for all threads to die..."
    killAsynchronousUpdaters()
    #time.sleep(1)
    while self.print_threads() > 0:         
        print "\nStill waiting for all threads to die..."
        time.sleep(1)
    print

  def reconfigure(self, config, level):
    # print "Reconfigure", config
    # Reconfigure the projector.
    self.projector.process_update(config, level)
    self.prosilica_inhibit.process_update(config, level)
    # Reconfigure the cameras.
    for camera in self.cameras.values():
      camera.process_update(config, level)
    #for trig_controller in self.trig_controllers:
    #  trig_controller.update()
    #for camera in self.cameras.keys():
    #  camera.update()
    for controller in self.controllers:
        controller.update();
    for camera in self.cameras.values():
        camera.apply_update()
    #print config
    self.config = config
    return config
  
  def update_diagnostics(self):
    da = DiagnosticArray()
    ds = DiagnosticStatus()
    ds.name = rospy.get_caller_id().lstrip('/') + ": Tasks"
    in_progress = 0;
    longest_interval = 0;
    for updater in list(asynchronous_updaters):
        (name, interval) = updater.getStatus()
        if interval == 0:
            msg = "Idle"
        else:
            in_progress = in_progress + 1
            msg = "Update in progress (%i s)"%interval
        longest_interval = max(interval, longest_interval)
        ds.values.append(KeyValue(name, msg))
    if in_progress == 0:
        ds.message = "Idle"
    else:
        ds.message = "Updates in progress: %i"%in_progress
    if longest_interval > 10:
        ds.level = 1
        ds.message = "Update is taking too long: %i"%in_progress
    ds.hardware_id = "none"
    da.status.append(ds)
    da.header.stamp = rospy.get_rostime()
    self.diagnostic_pub.publish(da)

  def spin(self):
    self.diagnostic_pub = rospy.Publisher("/diagnostics", DiagnosticArray)
    try:
      reset_count = 0
      rospy.loginfo("Camera synchronizer is running...")
      controller_update_count = 0
      while not rospy.is_shutdown():
          if self.config['camera_reset'] == True:
              reset_count = reset_count + 1
              if reset_count > 3:
                  self.server.update_configuration({'camera_reset' : False})
          else:
              reset_count = 0
          self.update_diagnostics()
          # In case the controllers got restarted, refresh their state.
          controller_update_count += 1
          if controller_update_count >= 10:
              controller_update_count = 0
              for controller in self.controllers:
                  controller.update();
          rospy.sleep(1)
    finally:
      rospy.signal_shutdown("Main thread exiting")
      self.kill()
      print "Main thread exiting"
