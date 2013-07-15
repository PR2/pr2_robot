#! /usr/bin/env python

import roslib
roslib.load_manifest('pr2_ethercat')
import time
import rospy, sys
import std_srvs.srv
from std_msgs.msg import Bool

halted = True
def callback(msg):
  global halted
  print "halted: %s" % msg.data
  halted = msg.data


rospy.init_node("test_halt")
rospy.Subscriber("motor_state", Bool, callback)
reset = rospy.ServiceProxy("pr2_ethercat/reset_motors", std_srvs.srv.Empty)
halt = rospy.ServiceProxy("pr2_ethercat/halt_motors", std_srvs.srv.Empty)
time.sleep(1)
print "Entering main loop: motors %s" % "halted" if halted else "running"
while 1:
  old_halted = halted
  if old_halted:
    print "Resetting motors"
    reset()
  else:
    print "Halting motors"
    halt()
  start = time.time()
  while old_halted == halted:
    time.sleep(0.1)
    if time.time() - start > 1:
      break
  if old_halted == halted:
    print "failed! old=%s new=%s" % (old_halted, halted) 
    break
  

