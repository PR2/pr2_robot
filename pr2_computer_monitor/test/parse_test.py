#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

##\author Kevin Watts

from __future__ import with_statement

PKG = 'pr2_computer_monitor'

import roslib; roslib.load_manifest(PKG)
import unittest

import pr2_computer_monitor

import os, sys

TEXT_PATH = 'test/sample_output/nvidia_smi_out.txt'
TEXT_HIGH_TEMP_PATH = 'test/sample_output/nvidia_smi_high_temp.txt'


##\brief Parses launch, tests.xml and configs.xml files in qualification
class TestNominalParser(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(roslib.packages.get_pkg_dir('pr2_computer_monitor'), TEXT_PATH), 'r') as f:
            self.data = f.read()

        with open(os.path.join(roslib.packages.get_pkg_dir('pr2_computer_monitor'), TEXT_HIGH_TEMP_PATH), 'r') as f:
            self.high_temp_data = f.read()

    def test_parse(self):
        gpu_stat = pr2_computer_monitor.parse_smi_output(self.data)
        
        # Check valid
        self.assert_(self.data, "Unable to read sample output, no test to run")

        # Check all string fields of message
        self.assert_(gpu_stat.pci_device_id, "No PCI Device ID found")
        self.assert_(gpu_stat.pci_location, "No PCI Location found")
        self.assert_(gpu_stat.display, "No Display found")
        self.assert_(gpu_stat.driver_version, "No Driver Version found")
        self.assert_(gpu_stat.product_name, "No Product Name found")

        self.assert_(gpu_stat.temperature > 40 and gpu_stat.temperature < 90, "Invalid temperature readings. Temperature: %d" % gpu_stat.temperature)
        self.assert_(gpu_stat.fan_speed > 0 and gpu_stat.fan_speed < 471, "Invalid fan speed readings. Fan Speed %f" % gpu_stat.fan_speed)

        diag_stat = pr2_computer_monitor.gpu_status_to_diag(gpu_stat)
        
        self.assert_(diag_stat.level == 0, "Diagnostics reports an error for nominal input. Message: %s" % diag_stat.message)

    def test_high_temp_parse(self):
        gpu_stat = pr2_computer_monitor.parse_smi_output(self.high_temp_data)
        
        # Check valid
        self.assert_(self.high_temp_data, "Unable to read sample output, no test to run")

        # Check all string fields of message
        self.assert_(gpu_stat.pci_device_id, "No PCI Device ID found")
        self.assert_(gpu_stat.pci_location, "No PCI Location found")
        self.assert_(gpu_stat.display, "No Display found")
        self.assert_(gpu_stat.driver_version, "No Driver Version found")
        self.assert_(gpu_stat.product_name, "No Product Name found")

        self.assert_(gpu_stat.temperature > 90, "Invalid temperature readings. Temperature: %d" % gpu_stat.temperature)
        self.assert_(gpu_stat.fan_speed > 0 and gpu_stat.fan_speed < 471, "Invalid fan speed readings. Fan Speed %s" % gpu_stat.fan_speed)

        diag_stat = pr2_computer_monitor.gpu_status_to_diag(gpu_stat)
        
        self.assert_(diag_stat.level == 1, "Diagnostics didn't report warning for high temp input. Level %d, Message: %s" % (diag_stat.level, diag_stat.message))


    def test_empty_parse(self):
        gpu_stat = pr2_computer_monitor.parse_smi_output('')
        
        self.assert_(gpu_stat.temperature == 0, "Invalid temperature reading. Should be 0. Reading: %d" % gpu_stat.temperature)
        
        diag_stat = pr2_computer_monitor.gpu_status_to_diag(gpu_stat)
        
        self.assert_(diag_stat.level == 2, "Diagnostics didn't reports an error for empty input. Level: %d, Message: %s" % (diag_stat.level, diag_stat.message))

        

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '-v':
        # Use to run tests verbosly
        suite = unittest.TestSuite()
        suite.addTest(TestNominalParser('test_parse'))
        suite.addTest(TestNominalParser('test_empty_parse'))
        suite.addTest(TestNominalParser('test_high_temp_parse'))
        
        unittest.TextTestRunner(verbosity = 2).run(suite)
    else:
        import rostest
        rostest.unitrun(PKG, 'parse_nominal', TestNominalParser)


