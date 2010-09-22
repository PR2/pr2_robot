#! /usr/bin/env python

import roslib
roslib.load_manifest('pr2_camera_synchronizer')
import rospy
import time

from ethercat_trigger_controllers.srv import SetMultiWaveform, SetMultiWaveformResponse
from ethercat_trigger_controllers.msg import MultiWaveform

class Main:
    def __init__(self):
        rospy.init_node('fake_multi_trigger_controller', anonymous = True)
        rospy.Service('~set_waveform', SetMultiWaveform, self.waveform_set)
        self.pub = rospy.Publisher('~waveform', MultiWaveform, latch = True)

    def waveform_set(self, req):
        self.pub.publish(req.waveform)
        return SetMultiWaveformResponse(success = True)

if __name__ == "__main__":
    Main()
    while not rospy.is_shutdown():
        time.sleep(0.1)
