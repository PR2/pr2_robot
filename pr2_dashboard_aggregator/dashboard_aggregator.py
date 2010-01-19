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

import roslib; roslib.load_manifest('pr2_dashboard_aggregator')

import time
import rospy
from std_msgs.msg import Bool
from pr2_msgs.msg import PowerBoardState, PowerState, DashboardState, AccessPoint

class DashboardAggregator:
  def __init__(self):
    self.msg = DashboardState()

    # Create publisher
    self.pub = rospy.Publisher("dashboard_agg", DashboardState)

    # Create subscribers
    # Circuit Breaker
    rospy.Subscriber("power_board/state", PowerBoardState, self.powerBoardCB)
    self.last_power_board_state = 0
    # Battery
    rospy.Subscriber("power_state", PowerState, self.powerCB)
    self.last_power_state = 0
    # Wireless
    rospy.Subscriber("ddwrt/accesspoint", AccessPoint, self.accessPointCB)
    self.last_access_point = 0
    # Motor State
    rospy.Subscriber("pr2_etherCAT/motors_halted", Bool, self.motorsHaltedCB)
    self.last_motors_halted = 0

  def motorsHaltedCB(self, msg):
    self.last_motors_halted = time.time()
    self.msg.motors_halted = msg

  def powerBoardCB(self, msg):
    self.last_power_board_state = time.time()
    self.msg.power_board_state = msg

  def powerCB(self, msg):
    self.last_power_state = time.time()
    self.msg.power_state = msg

  def accessPointCB(self, msg):
    self.last_access_point = time.time()
    self.msg.access_point = msg

  def publish(self):
    now = time.time()
    self.msg.motors_halted_valid = (now - self.last_motors_halted) < 3
    self.msg.power_board_state_valid = (now - self.last_power_board_state) < 3
    self.msg.power_state_valid = (now - self.last_power_state) < 25
    self.msg.access_point_valid = (now - self.last_access_point) < 5
    self.pub.publish(self.msg)

def main():
  rospy.init_node("pr2_dashboard_aggregator")
  da = DashboardAggregator()
  r = rospy.Rate(1)
  while not rospy.is_shutdown():
    da.publish()
    r.sleep()

if __name__ == "__main__":
  main()
